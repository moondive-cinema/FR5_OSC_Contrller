#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 OSC Bridge + JOG 통합 버전

기능:
- 슬롯 이동: /robot/move [slot:int 0~9]
- JOG: /fr5/jog/x|y|z|rx|ry|rz|yaw (Press=±1.0, Release=0.0)
- JOG 속도:
    - /fr5/jog/vel/{10|20|40|60|95}
    - /fr5/jog/vel <number>
    - /fr5/jog/vel/get
- 텔레메트리: /r/joint, /r/tcp, /r/tcp_speed (10Hz)
- 상태/알람:
    - /vp/robot_operation [current_slot, target_slot, moving, arrived]
    - /vp/robot_status    [system_state, heartbeat_count]
    - /vp/robot_alarm     [code, message]
- UI 피드백:
    - 슬롯: /ui/slot/{0..9} 0/1, /ui/slot/value int
    - JOG 속도: /ui/vel/{10|20|40|60|95} 0/1, /ui/vel/value float
- 종료: 홈 복귀 시도 → 슬롯 LED OFF → JOG 속도 LED OFF → RobotEnable(0) + CloseRPC()
"""

import os
import sys
import json
import time
import threading
import traceback
from functools import partial

# ---------------- Fairino SDK 경로 ----------------
SDK_ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    'fairino-python-sdk-main', 'windows', 'fairino'
))
if SDK_ROOT not in sys.path:  # 경로 중복 추가 방지
    sys.path.insert(0, SDK_ROOT)

try:
    import Robot as FRRobot
    print("✅ SDK import 성공")
except Exception as e:
    print(f"🛑 SDK import 실패: {e}")
    traceback.print_exc()
    sys.exit(1)

# ---------------- python-osc ----------------------
try:
    from pythonosc import dispatcher, osc_server, udp_client
    print("✅ python-osc import 성공")
except ImportError:
    print("🛑 python-osc 라이브러리 필요: pip install python-osc")
    sys.exit(1)

# ---------------- 설정 ---------------------------
ROBOT_IP        = "192.168.2.57"

AXIM_IP         = "192.168.2.50"
AXIM_PORT       = 7000
DB_FILE         = "fr5_presets.json"

OSC_LISTEN_IP   = "0.0.0.0"
OSC_LISTEN_PORT = 9001

# === V1에서 가져온 이동 속도 (슬롯 이동용) ===
MOVE_VEL_PERCENT = 50.0  # 필요시 조절

# === UI/Companion Feedback (Slot + JOG LEDs) ===
OSC_FEEDBACK_IP   = "127.0.0.1"
OSC_FEEDBACK_PORT = 9003

SLOTS = {
    0: "home",
    1: "cam1", 2: "cam2", 3: "cam3", 4: "cam4",
    5: "cam5", 6: "cam6", 7: "cam7", 8: "cam8", 9: "cam9"
}

# ---------------- 전역 (OSC 클라이언트들) ----------------
ax = udp_client.SimpleUDPClient(AXIM_IP, AXIM_PORT)          # Aximmetry용
ui = udp_client.SimpleUDPClient(OSC_FEEDBACK_IP, OSC_FEEDBACK_PORT)  # Companion LED용

# === V1 전역 변수 ===
current_slot    = 0
moving_flag     = 0
target_slot_ui  = 0
stop_flag       = False
seq_counter     = 0
last_alarm_code = None
move_lock = threading.Lock()
arrived_pulse_thread = None
arrived_pulse_cancel = threading.Event()

# === V1 알람 코드 ===
ALARM_OK        = 0
ALARM_ESTOP     = 1
ALARM_COLLISION = 2
ALARM_PROG_STOP = 3
ALARM_COMM_LOST = 4
ALARM_MOTIONFAIL= 5
ALARM_BUSY      = 6

# -------------------------------------------------
# JOG 관련 설정 (기존 JOG 스크립트에서 그대로 이동)
# -------------------------------------------------

# ---- 조그 참조 프레임 (ref/stop_ref): joint(0,1), base(2,3), tool(4,5), work(8,9)
JOG_REF_MOVE = 2   # base=2
JOG_REF_STOP = 3   # base=3

# ---- 조그 파라미터 ----
JOG_MAX_DIS  = 250.0   # mm/deg — 한 번의 StartJOG 최대 이동량
JOG_VEL_PCT  = 40.0    # 0~100 — 시작 기본 속도 40%
JOG_ACC_PCT  = 40.0    # 0~100 — 조그 가속 비율 (촬영용, 고정)
VEL_PRESETS  = (10, 20, 40, 60, 95)

AXIS_NB = {
    "x": 1, "y": 2, "z": 3,
    "rx": 4, "ry": 5, "rz": 6,
    "yaw": 6  # alias
}

# JOG 상태
jog_lock = threading.Lock()
jog_is_moving = False


# =================================================
# 공용 유틸 (DB, 상태 확인 등) - V1 + 일부 JOG용
# =================================================

def load_db(path):
    """V1 버전 사용 (경로 없으면 경고 후 빈 dict 반환)"""
    if not os.path.exists(path):
        print(f"⚠️ DB 없음: {path}")
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"⚠️ DB 로드 실패: {e}")
        return {}


def probe_comm_ok(robot):
    """V1 버전 사용"""
    try:
        _ = robot.robot_state_pkg.second
        return 1
    except Exception:
        return 0


# ---- 오퍼레이션 메시지 (V1) ----
def send_robot_operation(cur: int, tgt: int, moving: int, arrived: int):
    """V1 버전 사용"""
    try:
        ax.send_message("/vp/robot_operation",
                        [int(cur), int(tgt), int(moving), int(arrived)])
    except Exception as e:
        print(f"⚠️ robot_operation 전송 실패: {e}")


# ---- 도착 펄스 (V1) ----
def _pulse_arrived_worker(slot: int, duration: float, hz: int):
    """V1 버전 사용"""
    interval = 1.0 / float(hz)
    t_end = time.time() + max(0.05, duration)
    try:
        while time.time() < t_end:
            if arrived_pulse_cancel.is_set():
                return
            send_robot_operation(slot, slot, 0, 1)  # arrived=1
            time.sleep(interval)
        send_robot_operation(slot, slot, 0, 0)  # 마지막 0 프레임
    except Exception as e:
        print(f"🛑 _pulse_arrived_worker 예외: {e}")
        try:
            send_robot_operation(slot, slot, 0, 0)
        except Exception:
            pass


def pulse_arrived_async(slot: int, duration: float = 1.0, hz: int = 20):
    """V1 버전 사용"""
    global arrived_pulse_thread, arrived_pulse_cancel
    if arrived_pulse_thread and arrived_pulse_thread.is_alive():
        arrived_pulse_cancel.set()
        arrived_pulse_thread.join(timeout=0.3)
    arrived_pulse_cancel.clear()
    arrived_pulse_thread = threading.Thread(
        target=_pulse_arrived_worker, args=(slot, duration, hz), daemon=True
    )
    arrived_pulse_thread.start()


# ---- 상태/HB/알람 (V1) ----
def send_robot_status(system_state: int, seq: int):
    """V1 버전 사용"""
    try:
        ax.send_message("/vp/robot_status", [int(system_state), int(seq)])
    except Exception as e:
        print(f"⚠️ robot_status 전송 실패: {e}")


def send_alarm(code: int, message: str):
    """V1 버전 사용"""
    global last_alarm_code
    # 동일 알람 중복 방지
    if code == last_alarm_code and code != ALARM_OK:
        return
    try:
        ax.send_message("/vp/robot_alarm", [int(code), str(message)])
        last_alarm_code = code
    except Exception as e:
        print(f"⚠️ robot_alarm 전송 실패: {e}")


# ---- 도착 대기 (V1) ----
def wait_until_arrived(robot, poll_interval=0.05, timeout_s=120.0):
    """V1 버전 사용"""
    start = time.time()
    while (time.time() - start) < timeout_s:
        try:
            _ = robot.robot_state_pkg.second
        except Exception:
            return (False, "COMM_LOST")

        estop     = int(getattr(robot.robot_state_pkg, "EmergencyStop", 0))
        collision = int(getattr(robot.robot_state_pkg, "collisionState", 0))
        main_code = int(getattr(robot.robot_state_pkg, "main_code", 0))
        motion_dn = int(getattr(robot.robot_state_pkg, "motion_done", 0))

        if estop == 1:
            return (False, "E_STOP")
        if collision == 1:
            return (False, "COLLISION")
        if main_code != 0:
            return (False, f"MAIN_CODE_{main_code}")
        if motion_dn == 1:
            return (True, "OK")  # 도착!

        time.sleep(poll_interval)
    return (False, "TIMEOUT")  # 타임아웃


# ---- Companion 슬롯 LED 피드백 ----
def _emit_slot_leds(selected: int):
    """선택된 슬롯의 LED만 ON, 나머지는 OFF."""
    try:
        keys = sorted(SLOTS.keys())
        for k in keys:
            ui.send_message(f"/ui/slot/{k}", 1 if k == int(selected) else 0)
        ui.send_message("/ui/slot/value", int(selected))
        print(f"[UI] SLOT LED update: selected={selected}")
    except Exception as e:
        print(f"[WRN] _emit_slot_leds 예외: {e}")


def _emit_slot_leds_off():
    """모든 슬롯 LED OFF."""
    try:
        keys = sorted(SLOTS.keys())
        for k in keys:
            ui.send_message(f"/ui/slot/{k}", 0)
        ui.send_message("/ui/slot/value", -1)
        print("[UI] SLOT LED OFF (all)")
    except Exception as e:
        print(f"[WRN] _emit_slot_leds_off 예외: {e}")


# =================================================
# 텔레메트리/상태 스레드 (V1 그대로)
# =================================================

def telemetry_loop(robot):
    global stop_flag, seq_counter, current_slot, moving_flag, target_slot_ui, last_alarm_code

    t_pose = t_hb = 0.0
    prev_estop = None
    prev_collision = None

    if not hasattr(telemetry_loop, "_last_comm_ok"):
        telemetry_loop._last_comm_ok = None
    if not hasattr(telemetry_loop, "_last_system_state"):
        telemetry_loop._last_system_state = None
    if not hasattr(telemetry_loop, "_last_hb_interval"):
        telemetry_loop._last_hb_interval = None

    while not stop_flag:
        now = time.time()

        if now - t_pose >= 0.1:  # 10Hz 텔레메트리
            try:
                q = [float(robot.robot_state_pkg.jt_cur_pos[i]) for i in range(6)]
                tcp = [float(robot.robot_state_pkg.tl_cur_pos[i]) for i in range(6)]
                tcp_speed = [
                    float(robot.robot_state_pkg.actual_TCP_CmpSpeed[0]),
                    float(robot.robot_state_pkg.actual_TCP_CmpSpeed[1]),
                ]
                ax.send_message("/r/joint", q)
                ax.send_message("/r/tcp", tcp)
                ax.send_message("/r/tcp_speed", tcp_speed)
            except Exception as e:
                print(f"⚠️ 텔레메트리 실패: {e}")
            t_pose = now

        # 상태 평가
        comm_ok = probe_comm_ok(robot)
        if telemetry_loop._last_comm_ok is None:
            telemetry_loop._last_comm_ok = comm_ok
        elif telemetry_loop._last_comm_ok != comm_ok:
            print("🛑 통신 오류" if comm_ok == 0 else "✅ 통신 복구")
            telemetry_loop._last_comm_ok = comm_ok

        power_on      = int(getattr(robot.robot_state_pkg, "rbtEnableState", 0)) if comm_ok else 0
        program_state = int(getattr(robot.robot_state_pkg, "program_state", 0))   if comm_ok else 1
        motion_done   = int(getattr(robot.robot_state_pkg, "motion_done", 1))     if comm_ok else 1
        estop         = int(getattr(robot.robot_state_pkg, "EmergencyStop", 0))   if comm_ok else 0
        collision     = int(getattr(robot.robot_state_pkg, "collisionState", 0))  if comm_ok else 0
        main_code     = int(getattr(robot.robot_state_pkg, "main_code", 0))       if comm_ok else 0

        if comm_ok == 0 or estop == 1:
            system_state = 2  # 셧다운
        elif collision == 1 or main_code != 0:
            system_state = 1  # 경고
        else:
            system_state = 0  # 정상

        hb_interval = 5.0 if system_state in (1, 2) else (0.5 if moving_flag else 1.0)

        new_alarm = None
        if comm_ok == 0:
            new_alarm = (ALARM_COMM_LOST, "COMM_LOST")
        elif estop == 1:
            new_alarm = (ALARM_ESTOP, "E_STOP")
        elif collision == 1:
            new_alarm = (ALARM_COLLISION, "COLLISION")
        elif main_code != 0:
            new_alarm = (ALARM_PROG_STOP, f"PROGRAM_STOP(main_code={main_code})")
        else:
            new_alarm = (ALARM_OK, "OK")

        if new_alarm and (new_alarm[0] != last_alarm_code):
            send_alarm(new_alarm[0], new_alarm[1])

        send_immediately = (telemetry_loop._last_system_state != system_state)
        if send_immediately:
            seq_counter = (seq_counter + 1) & 0x7fffffff
            send_robot_status(system_state, seq_counter)
            telemetry_loop._last_system_state = system_state
            t_hb = now

        if now - t_hb >= hb_interval:
            seq_counter = (seq_counter + 1) & 0x7fffffff
            send_robot_status(system_state, seq_counter)
            if telemetry_loop._last_hb_interval != hb_interval:
                telemetry_loop._last_hb_interval = hb_interval
            t_hb = now

        if prev_estop is None:
            prev_estop, prev_collision = estop, collision
        else:
            if prev_estop != estop:
                print("🛑 E-STOP ON" if estop else "✅ E-STOP OFF")
                prev_estop = estop
            if prev_collision != collision:
                print("⚠️ COLLISION ON" if collision else "✅ COLLISION OFF")
                prev_collision = collision

        time.sleep(0.01)


# =================================================
# 슬롯 이동 핸들러 (기존 move_handler 그대로)
# =================================================

def move_handler(address, *args, robot=None, tool=0, user=0, slots=None, db_file=None):
    """ V1 기반 핸들러에 LED 업데이트 추가 """
    global current_slot, moving_flag, target_slot_ui

    if not args:
        print("⚠️ 인자 없음 (0~9 필요)")
        return
    try:
        target_slot = int(args[0])
    except ValueError:
        print(f"⚠️ 잘못된 슬롯 인자: {args[0]}")
        return
    if target_slot not in slots:
        print(f"⚠️ 유효하지 않은 슬롯: {target_slot}")
        return

    slot_name = slots[target_slot]
    db = load_db(db_file or DB_FILE)  # 기본 DB 파일 사용
    if not db or slot_name not in db:
        print(f"⚠️ '{slot_name}' 좌표 없음")
        return
    target_pose = db[slot_name].get("val")
    if not isinstance(target_pose, list) or len(target_pose) < 6:
        print(f"⚠️ '{slot_name}' 좌표값 불완전")
        return

    if not move_lock.acquire(blocking=False):
        print("🟡 이동 명령 거절: BUSY")
        send_alarm(ALARM_BUSY, "BUSY")
        return

    try:
        print(f"🚚 이동 시작 → '{slot_name}' (슬롯 {target_slot})")
        # LED 업데이트
        _emit_slot_leds(target_slot)

        if arrived_pulse_thread and arrived_pulse_thread.is_alive():
            arrived_pulse_cancel.set()
            arrived_pulse_thread.join(timeout=0.3)

        moving_flag    = 1
        target_slot_ui = target_slot
        send_robot_operation(current_slot, target_slot_ui, 1, 0)  # moving=1, arrived=0

        # MoveCart 호출
        err = 0
        try:
            # SDK 문서 3.9 MoveCart: desc_pos, tool, user, vel=20.0, ...
            err = robot.MoveCart(desc_pos=target_pose,
                                 tool=tool, user=user, vel=MOVE_VEL_PERCENT)
            if err is None:
                err = 0
        except TypeError as e:
            print(f"🛑 MoveCart 키워드 인자 호출 실패! ({e}) SDK 버전 불일치 가능성.")
            err = -1
        except Exception as e:
            print(f"🛑 MoveCart 호출 중 예외 발생: {e}")
            err = -2

        if err != 0:
            print(f"🛑 이동 명령 실패 (코드 {err})")
            moving_flag = 0
            target_slot_ui = current_slot
            send_robot_operation(current_slot, current_slot, 0, 0)
            send_alarm(ALARM_MOTIONFAIL, f"MOTION_FAIL(code={err})")
            return

        # 실제 도착 대기
        ok, reason = wait_until_arrived(robot, poll_interval=0.05, timeout_s=120.0)

        if ok:
            print(f"✅ 도착 확인 → '{slot_name}'")
            current_slot   = target_slot
            moving_flag    = 0
            target_slot_ui = current_slot
            pulse_arrived_async(current_slot, duration=1.0, hz=20)
            send_robot_operation(current_slot, target_slot_ui, 0, 0)
        else:
            print(f"🛑 도착 실패: {reason}")
            moving_flag = 0
            target_slot_ui = current_slot
            send_robot_operation(current_slot, current_slot, 0, 0)
            alarm_map = {
                "COMM_LOST": (ALARM_COMM_LOST, "COMM_LOST"),
                "E_STOP":    (ALARM_ESTOP, "E_STOP"),
                "COLLISION": (ALARM_COLLISION, "COLLISION"),
                "TIMEOUT":   (ALARM_MOTIONFAIL, "MOTION_TIMEOUT")
            }
            code, msg = alarm_map.get(reason, (ALARM_PROG_STOP, reason))
            send_alarm(code, msg)

    except Exception as e:
        print(f"🛑 move_handler 처리 중 예외: {e}")
        moving_flag = 0
        target_slot_ui = current_slot
        send_robot_operation(current_slot, current_slot, 0, 0)
    finally:
        move_lock.release()


# =================================================
# JOG 유틸/핸들러 (기존 JOG 스크립트 로직을 함수로 이식)
# =================================================

def jog_dump_state(robot, tag: str = ""):
    """기존 JOG _dump_state 동일 기능"""
    try:
        s = robot.robot_state_pkg
        fields = {
            "rbtEnableState": int(getattr(s, "rbtEnableState", -1)),
            "program_state":  int(getattr(s, "program_state", -1)),
            "EmergencyStop":  int(getattr(s, "EmergencyStop", -1)),
            "collisionState": int(getattr(s, "collisionState", -1)),
            "main_code":      int(getattr(s, "main_code", -1)),
            "motion_done":    int(getattr(s, "motion_done", -1)),
            "actual_TCP_CmpSpeed": list(getattr(s, "actual_TCP_CmpSpeed", [None, None])),
        }
        print(f"[DBG] STATE{(':'+tag) if tag else ''}: {fields}")
    except Exception as e:
        print("[WRN] state dump 실패:", e)


def jog_emit_speed_leds():
    """현재 JOG_VEL_PCT에 맞춰 프리셋 버튼 LED를 갱신."""
    if not ui:
        return
    try:
        tol = 0.5
        current = float(JOG_VEL_PCT)
        for p in VEL_PRESETS:
            on = 1 if abs(current - p) <= tol else 0
            ui.send_message(f"/ui/vel/{p}", on)
        ui.send_message("/ui/vel/value", current)
        print(
            f"[UI] JOG LED update: {current:.1f}% -> "
            + ", ".join(
                [f"{p}:{'ON' if abs(current-p)<=tol else 'off'}" for p in VEL_PRESETS]
            )
        )
    except Exception as e:
        print("[WRN] UI LED 갱신 실패:", e)


def jog_emit_speed_leds_off():
    """종료 시 모든 속도 LED OFF"""
    if not ui:
        return
    try:
        for p in VEL_PRESETS:
            ui.send_message(f"/ui/vel/{p}", 0)
        ui.send_message("/ui/vel/value", 0.0)
        print("[UI] JOG LED all OFF")
    except Exception as e:
        print("[WRN] UI LED OFF 송신 실패:", e)


def jog_set_vel_pct(pct: float):
    """기존 _set_jog_vel_pct 로직"""
    global JOG_VEL_PCT, jog_is_moving
    try:
        with jog_lock:
            if jog_is_moving:
                print(
                    f"[CFG] 🚫 속도 변경 무시 (이동 중) 요청={pct}% / 현재={JOG_VEL_PCT}%"
                )
                return
            old = JOG_VEL_PCT
            v = max(0.0, min(100.0, float(pct)))
            JOG_VEL_PCT = v
            print(
                f"[CFG] ✅ JOG_VEL_PCT: {old:.1f}% -> {JOG_VEL_PCT:.1f}% "
                f"(acc={JOG_ACC_PCT}%, global=30%)"
            )
    except Exception as e:
        print("[ERR] set_jog_vel_pct:", e)
        return

    # Companion/패널 LED 갱신
    jog_emit_speed_leds()


def jog_vel_arg(addr, *args):
    """ /fr5/jog/vel <number> 용 핸들러 """
    if not args:
        print("[CFG] ⚠️ /fr5/jog/vel 인자 없음")
        return
    try:
        val = float(args[0])
    except Exception as e:
        print(f"[CFG] ⚠️ /fr5/jog/vel 인자 변환 실패: {args} ({e})")
        return
    jog_set_vel_pct(val)


def jog_get_vel(addr, *args):
    """ /fr5/jog/vel/get 핸들러 """
    print(
        f"[CFG] ℹ️ JOG_VEL_PCT = {JOG_VEL_PCT:.1f}% "
        f"(acc={JOG_ACC_PCT}%, global=30%)"
    )
    if ui:
        ui.send_message("/ui/vel/value", float(JOG_VEL_PCT))


def jog_cb(addr, val, axis, robot):
    """ /fr5/jog/{axis} 핸들러, 기존 _cb_jog 로직 """
    global jog_is_moving
    print(f"[DBG] → jog_cb: axis={axis}, val={val}")
    try:
        val = float(val)
    except Exception as e:
        print("[ERR] jog_cb: val float 변환 실패:", e)
        traceback.print_exc()
        return

    jog_dump_state(robot, tag="pre")

    if axis not in AXIS_NB:
        print(f"[WRN] 미지원 축: {axis}")
        return

    try:
        if abs(val) < 1e-6:
            # STOP
            print(f"[DBG] Stop 요청 → StopJOG({JOG_REF_STOP})")
            rc = robot.StopJOG(JOG_REF_STOP)
            print(f"[DBG] StopJOG 리턴값: {rc}")
            print(f"[JOG] {axis:<3} STOP")
            jog_is_moving = False
            jog_dump_state(robot, tag="post")
            return

        # START
        nb = AXIS_NB[axis]
        dir_val = 1 if val > 0 else 0  # 0=음, 1=양 (매뉴얼 규약)
        print(
            f"[DBG] Start 요청 → StartJOG({JOG_REF_MOVE}, nb={nb}, dir={dir_val}, "
            f"max={JOG_MAX_DIS}, vel={JOG_VEL_PCT}, acc={JOG_ACC_PCT})"
        )

        rc = robot.StartJOG(JOG_REF_MOVE, nb, dir_val, JOG_MAX_DIS,
                            JOG_VEL_PCT, JOG_ACC_PCT)
        print(f"[DBG] StartJOG 리턴값: {rc}")
        if rc != 0:
            print(
                f"[ERR] 🛑 StartJOG 실패: err={rc} "
                f"(axis={axis}, nb={nb}, dir={dir_val})"
            )
            return

        jog_is_moving = True
        print(
            f"[JOG] {axis:<3} {'+' if dir_val else '-'}  max={JOG_MAX_DIS} "
            f"vel={JOG_VEL_PCT}% acc={JOG_ACC_PCT}% (ref={JOG_REF_MOVE})"
        )
        jog_dump_state(robot, tag="post")

    except Exception as e:
        print(f"[ERR] ⚠️ jog_cb 실행 중 예외: {e}")
        traceback.print_exc()


# =================================================
# 안전 종료 (슬롯 + JOG 모두 정리)
# =================================================

def shutdown(robot, server):
    global stop_flag, current_slot
    print("\n🔌 종료 중...")
    stop_flag = True

    # --- 홈 복귀 시도 (슬롯 스크립트 로직) ---
    db = load_db(DB_FILE)
    home_pose = db.get("home", {}).get("val")
    if isinstance(home_pose, list) and len(home_pose) >= 6:
        print("  ➡️ 홈 위치로 이동 시도...")
        try:
            tool_idx = int(getattr(robot.robot_state_pkg, "tool", 1))
            user_idx = int(getattr(robot.robot_state_pkg, "user", 0))
            err = robot.MoveCart(desc_pos=home_pose,
                                 tool=tool_idx, user=user_idx,
                                 vel=MOVE_VEL_PERCENT)
            if err == 0 or err is None:
                print("  ✅ 홈 위치 이동 완료 (또는 이미 홈)")
                current_slot = 0
                _emit_slot_leds(0)
            else:
                print(f"  ⚠️ 홈 위치 이동 실패 (코드: {err})")
        except Exception as e:
            print(f"  ⚠️ 홈 위치 이동 중 예외: {e}")
    else:
        print("  ⚠️ 홈 좌표(DB의 'home') 없음/불완전하여 홈 복귀 생략")

    # --- LED 끄기: 슬롯 + JOG ---
    _emit_slot_leds_off()
    jog_emit_speed_leds_off()

    # --- JOG 정지 ---
    try:
        robot.StopJOG(JOG_REF_STOP)
    except Exception as e:
        print("[WRN] StopJOG 중 예외:", e)

    # --- 텔레메트리 스레드 정리 및 OSC 서버 종료 ---
    if arrived_pulse_thread and arrived_pulse_thread.is_alive():
        arrived_pulse_cancel.set()
        arrived_pulse_thread.join(timeout=0.2)
    try:
        server.shutdown()
        server.server_close()
        print("  ✅ OSC 서버 종료됨.")
    except Exception as e:
        print(f"  ⚠️ OSC 서버 종료 오류: {e}")

    # --- 로봇 세션 종료 ---
    try:
        if hasattr(robot, 'RobotEnable'):
            robot.RobotEnable(0)
        if hasattr(robot, 'CloseRPC'):
            robot.CloseRPC()
            print("  ✅ 로봇 세션 정상 종료")
    except Exception as e:
        print(f"  ⚠️ 로봇 종료 오류: {e}")

    sys.exit(0)


# =================================================
# 메인: 슬롯 + JOG 핸들러 모두 Dispatcher에 연결
# =================================================

def main():
    global current_slot

    # --- 로봇 연결 ---
    try:
        robot = FRRobot(ROBOT_IP)
        print(f"🤖 FR5 연결됨 → {ROBOT_IP}")
    except Exception as e:
        print(f"🛑 연결 실패: {e}")
        sys.exit(1)

    # --- 초기화: Enable, Mode, DragTeach, SetSpeed(30) ---
    try:
        if hasattr(robot, 'RobotEnable'):
            robot.RobotEnable(1)
            print("✅ RobotEnable(1)")
        if hasattr(robot, 'Mode'):
            robot.Mode(0)  # Auto 모드
            print("✅ Mode(0)")
        if hasattr(robot, 'DragTeachSwitch'):
            robot.DragTeachSwitch(0)
            print("✅ DragTeachSwitch(0)")
        # JOG 스크립트에서 사용하던 전역 속도 스케일
        if hasattr(robot, 'SetSpeed'):
            try:
                rc = robot.SetSpeed(30.0)
                print(f"✅ SetSpeed(30.0) → {rc}")
            except Exception as e2:
                print(f"⚠️ SetSpeed 예외: {e2}")
        time.sleep(1)
        print("✅ 로봇 자동 모드 활성화 완료")
    except Exception as e:
        print(f"⚠️ 로봇 상태 설정 오류: {e}")

    # --- 좌표계 (슬롯 이동용 Tool/User) ---
    try:
        TOOL_IDX = int(getattr(robot.robot_state_pkg, "tool", 1))
        USER_IDX = int(getattr(robot.robot_state_pkg, "user", 0))
        print(f"✅ 좌표계: Tool={TOOL_IDX}, User={USER_IDX}")
    except Exception:
        TOOL_IDX, USER_IDX = 1, 0
        print("⚠️ 좌표계 감지 실패 → 기본값 사용 (T=1, U=0)")

    # --- 텔레메트리 스레드 시작 ---
    th = threading.Thread(target=telemetry_loop, args=(robot,), daemon=True)
    th.start()

    # --- Dispatcher 구성 ---
    disp = dispatcher.Dispatcher()

    # 1) 슬롯 이동 핸들러
    handler_move = partial(
        move_handler,
        robot=robot,
        tool=TOOL_IDX,
        user=USER_IDX,
        slots=SLOTS,
        db_file=DB_FILE
    )
    disp.map("/robot/move", handler_move)
    disp.map("/ping", lambda a, *b: print("✅ /ping 수신"))

    # 2) JOG 축별 핸들러
    for axis in AXIS_NB.keys():
        path = f"/fr5/jog/{axis}"
        # partial로 axis, robot 주입
        h = partial(jog_cb, axis=axis, robot=robot)
        disp.map(path, h)
        print(f"[DBG] JOG map 등록: {path} -> jog_cb(axis='{axis}')")

    # 3) JOG 속도 프리셋 핸들러
    def _mk_vel_handler(preset):
        def _h(addr, *args):
            jog_set_vel_pct(preset)
        return _h

    for preset in VEL_PRESETS:
        p = f"/fr5/jog/vel/{preset}"
        disp.map(p, _mk_vel_handler(preset))
        print(f"[DBG] JOG map 등록: {p} -> set vel {preset}%")

    # 4) JOG 숫자 인자/조회
    disp.map("/fr5/jog/vel", jog_vel_arg)
    disp.map("/fr5/jog/vel/get", jog_get_vel)

    # --- OSC 서버 ---
    server = osc_server.ThreadingOSCUDPServer(
        (OSC_LISTEN_IP, OSC_LISTEN_PORT), disp
    )

    # --- 시작 시 LED 초기화 (슬롯 0 + JOG 속도 40%) ---
    try:
        _emit_slot_leds(0)  # 홈 슬롯 LED
        current_slot = 0
    except Exception as e:
        print(f"⚠️ 초기 슬롯 LED 설정 중 오류: {e}")

    try:
        jog_emit_speed_leds()  # 현재 JOG_VEL_PCT(40%) 기반 JOG LED
    except Exception as e:
        print(f"⚠️ 초기 JOG 속도 LED 설정 중 오류: {e}")

    # --- 안내 및 서버 시작 ---
    try:
        print(f"\n--- 🤖 FR5 OSC Bridge + JOG 통합 ---")
        print(f"  수신 대기: {server.server_address[0]}:{server.server_address[1]}")
        print(f"  로봇 IP: {ROBOT_IP} (T:{TOOL_IDX}, U:{USER_IDX})")
        print(f"  Aximmetry: {AXIM_IP}:{AXIM_PORT}")
        print(f"  UI 피드백: {OSC_FEEDBACK_IP}:{OSC_FEEDBACK_PORT}")
        print(f"  명령(이동): /robot/move [0~9]")
        print(f"  명령(JOG):  /fr5/jog/x|y|z|rx|ry|rz|yaw  (±1.0 / 0.0)")
        print(f"  JOG 속도:  /fr5/jog/vel/{{10|20|40|60|95}} 또는 /fr5/jog/vel <number>")
        print(f"  텔레메트리: /r/joint, /r/tcp, /r/tcp_speed (10Hz)")
        print(f"  상태/HB:    /vp/robot_status, /vp/robot_operation, /vp/robot_alarm")
        print("  종료: Ctrl+C\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 종료 감지")
    finally:
        shutdown(robot, server)


if __name__ == "__main__":
    main()
