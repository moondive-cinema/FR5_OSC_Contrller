#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 OSC Bridge — Robot Operation / System Status / Unified Alarm + Telemetry
- /robot/move [slot:int]
- /vp/robot_operation [current_slot, target_slot, moving, arrived]
- /vp/robot_status    [system_state, heartbeat_count]    # HB 가변 (shutdown도 0.2Hz 전송)
- /vp/robot_alarm     [code, message]
- /r/joint, /r/tcp, /r/tcp_speed (10Hz)

HB 규칙 (수정됨)
- 정상 대기: 1 Hz (power_on=0, program_state=1 포함)
- 이동 중:   2 Hz
- 경고:      0.2 Hz (Collision, Main-Error)
- 셧다운:    0.2 Hz (E-Stop, Comm-Lost)
"""

import os, sys, json, time, threading
from functools import partial

# ---------------- Fairino SDK 경로 ----------------
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         'fairino-python-sdk-main', 'windows', 'fairino'))
sys.path.append(SDK_ROOT)
try:
    from Robot import RPC as FRRobot
    print("✅ SDK import 성공")
except Exception as e:
    print(f"🛑 SDK import 실패: {e}")
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

MOVE_VEL_PERCENT = 50.0

SLOTS = {
    0: "home",
    1: "cam1", 2: "cam2", 3: "cam3", 4: "cam4",
    5: "cam5", 6: "cam6", 7: "cam7", 8: "cam8", 9: "cam9"
}

# ---------------- 전역 ---------------------------
ax = udp_client.SimpleUDPClient(AXIM_IP, AXIM_PORT)

current_slot    = 0     # 마지막 도착/대기 슬롯
moving_flag     = 0     # 0=정지, 1=이동중
target_slot_ui  = 0     # 항상 유효 슬롯: 이동 중엔 목표, 대기 시 current와 동일
stop_flag       = False # 스레드 종료 플래그

seq_counter     = 0     # 하트비트 카운터
last_alarm_code = None  # 같은 알람 중복 송신 방지

# 경합 방지
move_lock = threading.Lock()            # 이동 직렬화
arrived_pulse_thread = None             # 도착 펄스 스레드
arrived_pulse_cancel = threading.Event()  # 펄스 취소 플래그

# 알람 코드 (power_on 제거, prog_stop은 main_code 전용)
ALARM_OK        = 0
ALARM_ESTOP     = 1
ALARM_COLLISION = 2
ALARM_PROG_STOP = 3  # (main_code != 0)
ALARM_COMM_LOST = 4
ALARM_MOTIONFAIL= 5
ALARM_BUSY      = 6

# ---------------- 유틸 ---------------------------
def load_db(path):
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
    """상태 패킷의 간단한 필드 접근으로 통신 여부 판단."""
    try:
        _ = robot.robot_state_pkg.second
        return 1
    except Exception:
        return 0

# ---- 오퍼레이션 메시지 ----
def send_robot_operation(cur:int, tgt:int, moving:int, arrived:int):
    """
    /vp/robot_operation [current_slot, target_slot, moving, arrived]
    - cur, tgt는 항상 유효 슬롯 번호(0~9)
    - arrived=1 은 1초 펄스 (비동기), 평상시 0
    """
    try:
        ax.send_message("/vp/robot_operation", [int(cur), int(tgt), int(moving), int(arrived)])
    except Exception as e:
        print(f"⚠️ robot_operation 전송 실패: {e}")

def _pulse_arrived_worker(slot:int, duration:float, hz:int):
    """취소 가능 도착 펄스 워커(비동기). cancel 세트 시 즉시 종료."""
    interval = 1.0 / float(hz)
    t_end = time.time() + max(0.05, duration)
    while time.time() < t_end:
        if arrived_pulse_cancel.is_set():
            return
        send_robot_operation(slot, slot, 0, 1)
        time.sleep(interval)
    # 정상 종료 시 마지막 0 프레임
    send_robot_operation(slot, slot, 0, 0)

def pulse_arrived_async(slot:int, duration:float=1.0, hz:int=20):
    """기존 펄스가 있으면 취소하고 새 펄스를 비동기로 시작."""
    global arrived_pulse_thread, arrived_pulse_cancel
    if arrived_pulse_thread and arrived_pulse_thread.is_alive():
        arrived_pulse_cancel.set()
        arrived_pulse_thread.join(timeout=0.3)
    arrived_pulse_cancel.clear()
    arrived_pulse_thread = threading.Thread(
        target=_pulse_arrived_worker, args=(slot, duration, hz), daemon=True
    )
    arrived_pulse_thread.start()

# ---- 상태/HB/알람 ----
def send_robot_status(system_state:int, seq:int):
    try:
        ax.send_message("/vp/robot_status", [int(system_state), int(seq)])
    except Exception as e:
        print(f"⚠️ robot_status 전송 실패: {e}")

def send_alarm(code:int, message:str):
    try:
        ax.send_message("/vp/robot_alarm", [int(code), str(message)])
    except Exception as e:
        print(f"⚠️ robot_alarm 전송 실패: {e}")

# ---------------- 도착 대기 -----------------------
def wait_until_arrived(robot, poll_interval=0.05, timeout_s=120.0):
    """
    motion_done==1 이 될 때까지 대기.
    중간에 통신 끊김 / E-STOP / 충돌 / 메인 에러 감지 시 즉시 실패.
    """
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

        if estop == 1:      return (False, "E_STOP")
        if collision == 1:  return (False, "COLLISION")
        if main_code != 0:  return (False, f"MAIN_CODE_{main_code}")
        if motion_dn == 1:  return (True, "OK")

        time.sleep(poll_interval)

    return (False, "TIMEOUT")

# ---------------- 텔레메트리/상태 스레드 ----------------
def telemetry_loop(robot):
    """
    10Hz: /r/joint, /r/tcp, /r/tcp_speed
    가변 HB + 상태: /vp/robot_status [system_state, seq]
      - 정상 대기: 1Hz (power_on=0, program_state=1 포함)
      - 이동 중:   2Hz
      - 경고:      0.2Hz (Collision, Main-Error)
      - 셧다운:    0.2Hz (E-Stop, Comm-Lost)
    상태 전환 시엔 즉시 1프레임을 추가로 송신.
    """
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

        # 10Hz 텔레메트리
        if now - t_pose >= 0.1:
            try:
                q = [float(robot.robot_state_pkg.jt_cur_pos[i]) for i in range(6)]
                tcp = [float(robot.robot_state_pkg.tl_cur_pos[i]) for i in range(6)]
                tcp_speed = [
                    float(robot.robot_state_pkg.actual_TCP_CmpSpeed[0]),  # mm/s
                    float(robot.robot_state_pkg.actual_TCP_CmpSpeed[1]),  # deg/s
                ]
                ax.send_message("/r/joint", q)
                ax.send_message("/r/tcp", tcp)
                ax.send_message("/r/tcp_speed", tcp_speed)
            except Exception as e:
                print(f"⚠️ 텔레메트리 전송 실패: {e}")
            t_pose = now

        # 상태 평가
        comm_ok = probe_comm_ok(robot)
        if telemetry_loop._last_comm_ok is None:
            telemetry_loop._last_comm_ok = comm_ok
        else:
            if telemetry_loop._last_comm_ok == 1 and comm_ok == 0:
                print("🛑 통신 오류: 상태 패킷 접근 실패(연결 끊김으로 추정)")
            elif telemetry_loop._last_comm_ok == 0 and comm_ok == 1:
                print("✅ 통신 복구")
            telemetry_loop._last_comm_ok = comm_ok

        # [참고] power_on, program_state는 HB 로그 출력을 위해 변수 자체는 남겨둠
        power_on     = int(getattr(robot.robot_state_pkg, "rbtEnableState", 0)) if comm_ok else 0
        program_state= int(getattr(robot.robot_state_pkg, "program_state", 0))   if comm_ok else 1
        motion_done  = int(getattr(robot.robot_state_pkg, "motion_done", 1))     if comm_ok else 1
        estop        = int(getattr(robot.robot_state_pkg, "EmergencyStop", 0))   if comm_ok else 0
        collision    = int(getattr(robot.robot_state_pkg, "collisionState", 0))  if comm_ok else 0
        main_code    = int(getattr(robot.robot_state_pkg, "main_code", 0))       if comm_ok else 0

        # --- [수정된 시스템 상태 결정] ---
        # power_on == 0, program_state == 1 조건 완전 제거
        if comm_ok == 0 or estop == 1:
            system_state = 2  # 셧다운 (Comm-Lost, E-Stop)
        elif collision == 1 or main_code != 0:
            system_state = 1  # 경고 (Collision, Main-Error)
        else:
            system_state = 0  # 정상
        # -------------------------------

        # HB 주기 (셧다운도 0.2Hz로 전송)
        if system_state in (1, 2):
            hb_interval = 5.0          # 0.2Hz
        else:
            hb_interval = 0.5 if (moving_flag == 1) else 1.0  # 이동 2Hz, 대기 1Hz

        # --- [수정된 알람 일원화] ---
        # program_state == 1 알람 블록 완전 제거
        new_alarm = None
        if comm_ok == 0:
            new_alarm = (ALARM_COMM_LOST, "COMM_LOST")
        elif estop == 1:
            new_alarm = (ALARM_ESTOP, "E_STOP")
        elif collision == 1:
            new_alarm = (ALARM_COLLISION, "COLLISION")
        # [수정] program_state == 1 알람 블록 완전 제거
        elif main_code != 0: 
            new_alarm = (ALARM_PROG_STOP, f"PROGRAM_STOP(main_code={main_code})")
        else:
            new_alarm = (ALARM_OK, "OK")
        # ---------------------------

        if new_alarm and (new_alarm[0] != last_alarm_code):
            send_alarm(new_alarm[0], new_alarm[1])
            last_alarm_code = new_alarm[0]

        # 상태 전환 시 즉시 1프레임 송신
        send_immediately = (telemetry_loop._last_system_state != system_state)
        if send_immediately:
            seq_counter = (seq_counter + 1) & 0x7fffffff
            send_robot_status(system_state, seq_counter)
            telemetry_loop._last_system_state = system_state
            t_hb = now  # 주기 타이머 리셋

        # 하트비트 주기 송신
        if now - t_hb >= hb_interval:
            seq_counter = (seq_counter + 1) & 0x7fffffff
            send_robot_status(system_state, seq_counter)

            # [수정] 주기가 변경될 때만 로그 출력
            if telemetry_loop._last_hb_interval != hb_interval:
                print(f"✅ [HB Change] s={system_state} -> {hb_interval:.1f}s interval. "
                      f"comm={comm_ok} power={power_on} prog={program_state} "
                      f"motion_done={motion_done} estop={estop} collision={collision} "
                      f"moving={moving_flag} cur={current_slot} tgt={target_slot_ui}")
                telemetry_loop._last_hb_interval = hb_interval
            
            t_hb = now

        # 엣지 로그
        if prev_estop is None:
            prev_estop = estop
            prev_collision = collision
        else:
            if prev_estop == 0 and estop == 1: print("🛑 [EDGE] E-STOP ON")
            if prev_estop == 1 and estop == 0: print("✅ [EDGE] E-STOP OFF")
            if prev_collision == 0 and collision == 1: print("⚠️ [EDGE] COLLISION ON")
            if prev_collision == 1 and collision == 0: print("✅ [EDGE] COLLISION OFF")
            prev_estop = estop
            prev_collision = collision

        time.sleep(0.01)

# --------------- 이동 명령 핸들러 ----------------
def move_handler(address, *args, robot=None, tool=0, user=0, slots=None, db_file=None):
    """
    /robot/move [slot:int] → MoveCart
    - 출발: current 유지, target=목표로 즉시 갱신, moving=1, arrived=0
    - 실제 도착(motion_done==1) 확인 후에만 current=target으로 바꾸고 arrived 펄스(비동기)
    - 이동 중 새 명령은 BUSY로 거절(안전). 원하면 큐/선점으로 변경 가능.
    """
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
    db = load_db(db_file)
    if not db or slot_name not in db:
        print(f"⚠️ '{slot_name}' 좌표 없음")
        return
    target_pose = db[slot_name].get("val")
    if not target_pose or len(target_pose) < 6:
        print(f"⚠️ '{slot_name}' 좌표값 불완전")
        return

    # 이동 직렬화: 이동 중이면 거절 (충돌 방지)
    if not move_lock.acquire(blocking=False):
        print("🟡 이동 명령 거절: 이전 이동 처리 중(BUSY)")
        send_alarm(ALARM_BUSY, "BUSY")
        return

    try:
        print(f"🚚 이동 시작 → '{slot_name}' (슬롯 {target_slot})")

        # 0) 기존 '도착 펄스'가 돌고 있으면 취소 및 정리
        if arrived_pulse_thread and arrived_pulse_thread.is_alive():
            arrived_pulse_cancel.set()
            arrived_pulse_thread.join(timeout=0.3)

        # 1) 출발: target 즉시 갱신, moving=1
        moving_flag    = 1
        target_slot_ui = target_slot
        send_robot_operation(current_slot, target_slot_ui, 1, 0)

        # 2) 이동 명령
        err = robot.MoveCart(desc_pos=target_pose, tool=tool, user=user, vel=MOVE_VEL_PERCENT)
        if err != 0:
            print(f"🛑 이동 명령 실패 (코드 {err})")
            moving_flag    = 0
            target_slot_ui = current_slot
            send_robot_operation(current_slot, current_slot, 0, 0)
            send_alarm(ALARM_MOTIONFAIL, f"MOTION_FAIL(code={err})")
            return

        # 3) 실제 도착 대기
        ok, reason = wait_until_arrived(robot, poll_interval=0.05, timeout_s=120.0)
        if ok:
            print(f"✅ 도착 확인 → '{slot_name}'")
            current_slot   = target_slot
            moving_flag    = 0
            target_slot_ui = current_slot  # 대기 시 target=current 정책
            # 도착 펄스: 비동기 (새 명령 시 즉시 취소 가능)
            pulse_arrived_async(current_slot, duration=1.0, hz=20)
        else:
            print(f"🛑 도착 실패: {reason}")
            moving_flag    = 0
            target_slot_ui = current_slot  # 실패 시 target=current로 복구
            send_robot_operation(current_slot, current_slot, 0, 0)
            
            # 알람 송신
            alarm_map = {
                "COMM_LOST":  (ALARM_COMM_LOST,  "COMM_LOST"),
                "E_STOP":     (ALARM_ESTOP,      "E_STOP"),
                "COLLISION":  (ALARM_COLLISION,  "COLLISION"),
                "TIMEOUT":    (ALARM_MOTIONFAIL, "MOTION_TIMEOUT"),
            }
            # [수정] ALARM_PROG_STOP이 reason에 대한 기본 알람이 됨
            code, msg = alarm_map.get(reason, (ALARM_PROG_STOP, reason)) 
            send_alarm(code, msg)

    finally:
        move_lock.release()

# ---------------- 안전 종료 ----------------------
def shutdown(robot, server):
    global stop_flag
    print("\n🔌 종료 중...")
    stop_flag = True
    try:
        server.shutdown()
        server.server_close()
        print("  ✅ OSC 서버 종료됨.")
    except Exception as e:
        print(f"  ⚠️ OSC 서버 종료 오류: {e}")
    try:
        robot.RobotEnable(0)
        robot.CloseRPC()
        print("  ✅ 로봇 세션 정상 종료")
    except Exception as e:
        print(f"  ⚠️ 로봇 종료 오류: {e}")
    sys.exit(0)

# ---------------- 메인 --------------------------
def main():
    try:
        robot = FRRobot(ROBOT_IP)
        print(f"🤖 FR5 연결됨 → {ROBOT_IP}")
    except Exception as e:
        print(f"🛑 연결 실패: {e}")
        sys.exit(1)

    try:
        robot.RobotEnable(1)
        robot.Mode(0)
        robot.DragTeachSwitch(0)
        time.sleep(1)
        print("✅ 로봇 자동 모드 활성화 완료")
    except Exception as e:
        print(f"⚠️ 로봇 상태 설정 오류: {e}")

    try:
        TOOL_IDX = int(getattr(robot.robot_state_pkg, "tool", 1))
        USER_IDX = int(getattr(robot.robot_state_pkg, "user", 0))
        print(f"✅ 좌표계: Tool={TOOL_IDX}, User={USER_IDX}")
    except Exception:
        TOOL_IDX, USER_IDX = 1, 0
        print("⚠️ 좌표계 감지 실패 → 기본값 사용 (T=1, U=0)")

    th = threading.Thread(target=telemetry_loop, args=(robot,), daemon=True)
    th.start()

    disp = dispatcher.Dispatcher()
    handler = partial(move_handler, robot=robot, tool=TOOL_IDX, user=USER_IDX,
                      slots=SLOTS, db_file=DB_FILE)
    disp.map("/robot/move", handler)
    disp.map("/ping", lambda a,*b: print("✅ /ping 수신"))

    server = osc_server.ThreadingOSCUDPServer((OSC_LISTEN_IP, OSC_LISTEN_PORT), disp)

    try:
        print(f"\n--- 🤖 FR5 OSC 브릿지 시작 ---")
        print(f"  수신 대기: {server.server_address[0]}:{server.server_address[1]}")
        print(f"  로봇 IP: {ROBOT_IP} (T:{TOOL_IDX}, U:{USER_IDX})")
        print(f"  Aximmetry: {AXIM_IP}:{AXIM_PORT}")
        print(f"  명령: /robot/move [0~9]")
        print(f"  오퍼레이션: /vp/robot_operation [current, target, moving, arrived]")
        print(f"  상태/HB:    /vp/robot_status [system_state, seq]  # shutdown도 0.2Hz 송신")
        print(f"  알람:       /vp/robot_alarm  [code, message]")
        print(f"  텔레메트리: /r/joint, /r/tcp, /r/tcp_speed (10Hz)")
        print("  종료: Ctrl+C\n")
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[Ctrl+C] 종료 감지")
    finally:
        shutdown(robot, server)

if __name__ == "__main__":
    main()