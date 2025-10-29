#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 OSC JOG — Prod build (no dryrun) + Companion feedback + Slot LED feedback
- 초기화: RobotEnable(1) → Mode(0) → DragTeachSwitch(0) → SetSpeed(30) → 1s wait
- 조그: /fr5/jog/x|y|z|rx|ry|rz  (Press=±1.0, Release=0.0)
- 속도: /fr5/jog/vel/{10|20|40|60|95}  또는 /fr5/jog/vel <number> (정지 상태에서만 반영)
- 전역 속도 30%, 조그 가속 40% 고정, 이동 중 속도 변경 금지
- Companion/TouchoOSC 등으로 피드백 송신:
    • /ui/vel/{10|20|40|60|95} = 0/1, /ui/vel/value = float
    • /ui/slot/0..9 = 0/1 (선택 슬롯 LED)
- 시작 기본 속도 40%, 조그 시작/종료 시 슬롯 LED OFF, 셧다운 시 모든 LED OFF

추가:
- /robot/move [0~9] 수신 시 해당 슬롯만 LED ON, 나머지는 OFF (이동 명령 자체는 별도 구현과 연동)
- 조그를 이용해 움직이기 시작하면 모든 슬롯 LED는 OFF
"""

import os, sys, time, threading, traceback
from pythonosc import dispatcher, osc_server
from pythonosc.udp_client import SimpleUDPClient

# ---------------- 사용자 설정 ----------------
ROBOT_IP        = os.environ.get("FR5_IP", "192.168.58.2")
OSC_LISTEN_IP    = "0.0.0.0"   # 이 서버가 받을 주소
OSC_LISTEN_PORT  = 9001        # 이 서버가 받을 포트 (Companion → Python 제어)
OSC_FEEDBACK_IP   = "127.0.0.1"  # Companion과 같은 PC면 127.0.0.1
OSC_FEEDBACK_PORT = 9003         # Companion 피드백 수신 포트 (Listen for Feedback = ON)

# 조그 파라미터
JOG_REF_MOVE = 0    # 0=joint, 1=cart(?) — SDK에 맞게 유지
JOG_REF_STOP = 0
JOG_MAX_DIS  = 250.0   # mm/deg — 한 번의 StartJOG 최대 이동량
JOG_VEL_PCT  = 40.0    # 0~100 — 시작 기본 속도 40%
JOG_ACC_PCT  = 40.0

VEL_PRESETS = [10, 20, 40, 60, 95]

# FR5 SDK import
try:
    from Robot import RPC as FRRobot
except Exception as e:
    print("[ERR] FRRobot SDK import 실패:", e)
    FRRobot = None

AXIS_NB = {
    "x":1, "y":2, "z":3, "rx":4, "ry":5, "rz":6,
    "yaw":6  # alias
}

class FR5JogServer:
    def __init__(self):
        self.lock = threading.Lock()
        self.robot = None
        self.rx_count = 0
        self.is_moving = False

        # UI/패널 피드백용 클라이언트
        try:
            self.ui = SimpleUDPClient(OSC_FEEDBACK_IP, OSC_FEEDBACK_PORT)
            print(f"[DBG] UI feedback client → {OSC_FEEDBACK_IP}:{OSC_FEEDBACK_PORT}")
        except Exception as e:
            self.ui = None
            print("[WRN] UI feedback client 생성 실패:", e)

        self._init_robot()
        self._init_osc()

        # 초기 LED 동기화 (현재 JOG_VEL_PCT 기준: 40%)
        self._emit_speed_leds()
        self.selected_slot = None
        self._emit_slot_leds_off()

    # ---------------- 로봇 초기화 ----------------
    def _init_robot(self):
        print(f"[DBG] 🔌 FR5 연결 시도 → {ROBOT_IP}")
        try:
            self.robot = FRRobot(ROBOT_IP)
            print("[DBG] 🤖 FR5 연결 객체 생성 완료:", self.robot)
        except Exception as e:
            print("[ERR] 🛑 FRRobot 연결 실패:", e)
            traceback.print_exc()
            sys.exit(1)

        # 자동 모드 및 속도 설정
        try:
            self.robot.RobotEnable(1)
            try:
                self.robot.Mode(0)  # Auto
            except Exception as e2:
                print("[WRN] Mode(0) 실패 (무시 가능):", e2)
            try:
                self.robot.DragTeachSwitch(0)
            except Exception as e2:
                print("[WRN] DragTeachSwitch(0) 무시:", e2)
            try:
                self.robot.SetSpeed(30)  # global speed
            except Exception as e2:
                print("[WRN] SetSpeed 예외:", e2)

            time.sleep(1)
            print("[DBG] ✅ 로봇 자동 모드 활성화 완료")
        except Exception as e:
            print("[ERR] ⚠️ 로봇 초기화 호출 중 예외:", e)
            traceback.print_exc()

        # 좌표계 상태(옵션)
        try:
            tool = int(getattr(self.robot.robot_state_pkg, "tool", 1))
            user = int(getattr(self.robot.robot_state_pkg, "user", 0))
            print(f"[DBG] ℹ️ 좌표계 상태: Tool={tool}, User={user}")
        except Exception as e:
            print("[WRN] ℹ️ 좌표계 읽기 실패:", e)

    # ---------------- OSC 초기화 ----------------
    def _init_osc(self):
        print("[DBG] dispatcher/서버 초기화")
        self.disp = dispatcher.Dispatcher()

        # ---- 축 조그 ----
        for axis in ["x","y","z","rx","ry","rz"]:
            path = f"/fr5/jog/{axis}"
            def make_handler(ax):
                def handler(addr, val):
                    return self._cb_jog(addr, val, ax)
                return handler
            self.disp.map(path, make_handler(axis))
            print(f"[DBG] map() 등록: {path} -> handler(ax='{axis}')")

        # ---- 속도 프리셋 버튼 (정지 상태에서만 반영) ----
        def _mk_vel_handler(preset):
            def _h(addr, *args):
                self._set_jog_vel_pct(preset)
            return _h

        for preset in VEL_PRESETS:
            p = f"/fr5/jog/vel/{preset}"
            self.disp.map(p, _mk_vel_handler(preset))
            print(f"[DBG] map() 등록: {p} -> set vel {preset}%")

        # ---- 숫자 인자 1개를 받는 일반 경로 매핑 (/fr5/jog/vel 60) ----
        self.disp.map("/fr5/jog/vel", self._cb_vel_arg)
        print("[DBG] map() 등록: /fr5/jog/vel -> set vel <number>")

        # (옵션) 현재 속도 조회
        self.disp.map("/fr5/jog/vel/get", self._get_vel)

        # 슬롯 선택 (LED 피드백)
        self.disp.map("/robot/move", self._cb_select_slot)
        print("[DBG] map() 등록: /robot/move -> select slot & LED feedback")

        # 서버 바인딩
        print(f"[DBG] OSC 서버 바인딩 시도: {OSC_LISTEN_IP}:{OSC_LISTEN_PORT}")
        try:
            self.server = osc_server.ThreadingOSCUDPServer(
                (OSC_LISTEN_IP, OSC_LISTEN_PORT), self.disp
            )
            print(f"[DBG] ✅ OSC 서버 리슨 중: {self.server.server_address}")
        except Exception as e:
            print("[ERR] 🛑 OSC 서버 시작 실패:", e)
            traceback.print_exc()
            sys.exit(1)

    # ---------------- 핸들러: 조그 ----------------
    def _dump_state(self, tag=""):
        try:
            st = self.robot.robot_state_pkg
            print(f"[{tag}] state: pwr={getattr(st,'power_state',None)} auto={getattr(st,'is_auto',None)} err={getattr(st,'error',None)}")
        except Exception:
            pass

    def _cb_jog(self, addr, val, axis:str):
        with self.lock:
            try:
                self._dump_state(tag="pre")

                if axis not in AXIS_NB:
                    print(f"[WRN] 미지원 축: {axis}")
                    return

                try:
                    if abs(val) < 1e-6:
                        # STOP
                        print(f"[DBG] Stop 요청 → StopJOG({JOG_REF_STOP})")
                        rc = self.robot.StopJOG(JOG_REF_STOP)
                        print(f"[DBG] StopJOG 리턴값: {rc}")
                        print(f"[JOG] {axis:<3} STOP")
                        self.is_moving = False
                        self._dump_state(tag="post")
                        return

                    # START
                    nb  = AXIS_NB[axis]
                    dir = 1 if val > 0 else 0
                    # 조그 시작 시 슬롯 LED 전부 OFF
                    self._emit_slot_leds_off()
                    print(f"[DBG] Start 요청 → StartJOG({JOG_REF_MOVE}, nb={nb}, dir={dir}, max={JOG_MAX_DIS}, vel={JOG_VEL_PCT}, acc={JOG_ACC_PCT})")
                    rc = self.robot.StartJOG(JOG_REF_MOVE, nb, dir, JOG_MAX_DIS,
                                             JOG_VEL_PCT, JOG_ACC_PCT)
                    print(f"[DBG] StartJOG 리턴값: {rc}")
                    if rc != 0:
                        print(f"[ERR] 🛑 StartJOG 실패: err={rc} (axis={axis}, nb={nb}, dir={dir})")
                        # 실패 시 이동 상태로 전환하지 않음
                        return

                    self.is_moving = True
                    print(f"[JOG] {axis:<3} {'+' if dir else '-'}  max={JOG_MAX_DIS} "
                          f"vel={JOG_VEL_PCT}% acc={JOG_ACC_PCT}% (ref={JOG_REF_MOVE})")
                    self._dump_state(tag="post")

                except Exception as e:
                    print(f"[ERR] ⚠️ _cb_jog 실행 중 예외: {e}")
                    traceback.print_exc()

            except Exception as e:
                print(f"[ERR] ⚠️ _cb_jog 바깥 예외: {e}")
                traceback.print_exc()

    # ---------------- 속도 프리셋/숫자 인자 처리 ----------------
    def _cb_vel_arg(self, addr, *args):
        # /fr5/jog/vel <number> 형태 지원 (예: 10, 20, 40, 60, 95 또는 임의 값)
        if not args:
            print("[CFG] ⚠️ /fr5/jog/vel 인자 없음")
            return
        try:
            val = float(args[0])
        except Exception as e:
            print(f"[CFG] ⚠️ /fr5/jog/vel 인자 변환 실패: {args} ({e})")
            return
        self._set_jog_vel_pct(val)

    def _set_jog_vel_pct(self, pct:float):
        global JOG_VEL_PCT
        try:
            with self.lock:
                if self.is_moving:
                    # 이동 중 변경 금지 (요청은 무시)
                    print(f"[CFG] 🚫 속도 변경 무시 (이동 중) 요청={pct}% / 현재={JOG_VEL_PCT}%")
                    return
                old = JOG_VEL_PCT
                v = max(0.0, min(100.0, float(pct)))
                JOG_VEL_PCT = v
                print(f"[CFG] ✅ JOG_VEL_PCT: {old:.1f}% -> {JOG_VEL_PCT:.1f}% "
                      f"(acc={JOG_ACC_PCT}%, global=30%)")
        except Exception as e:
            print("[ERR] set_jog_vel_pct:", e)
            return

        # 변경 성공 시 Companion/패널 LED 갱신
        self._emit_speed_leds()

    def _get_vel(self, addr, *args):
        print(f"[CFG] ℹ️ JOG_VEL_PCT = {JOG_VEL_PCT:.1f}% (acc={JOG_ACC_PCT}%, global=30%)")
        if getattr(self, "ui", None):
            self.ui.send_message("/ui/vel/value", float(JOG_VEL_PCT))

    # ---------------- 슬롯 LED 피드백 ----------------
    SLOT_MIN = 0
    SLOT_MAX = 9  # 0=Home, 1..9=Cams

    def _emit_slot_leds(self, selected:int):
        """선택한 슬롯 LED만 ON, 나머지는 OFF"""
        if not self.ui:
            return
        try:
            for s in range(self.SLOT_MIN, self.SLOT_MAX+1):
                self.ui.send_message(f"/ui/slot/{s}", 1 if s == selected else 0)
            print(f"[UI] SLOT LED: selected={selected}")
        except Exception as e:
            print("[WRN] SLOT LED 송신 실패:", e)

    def _emit_slot_leds_off(self):
        """모든 슬롯 LED OFF (조그 시작/셧다운 시)"""
        if not self.ui:
            return
        try:
            for s in range(self.SLOT_MIN, self.SLOT_MAX+1):
                self.ui.send_message(f"/ui/slot/{s}", 0)
            print("[UI] SLOT LED all OFF")
        except Exception as e:
            print("[WRN] SLOT LED OFF 송신 실패:", e)

    def _cb_select_slot(self, addr, *args):
        """슬롯 선택 요청: /robot/move [0~9] 수신 시 LED 피드백"""
        try:
            if not args:
                print("[CFG] ⚠️ /robot/move 인자 없음")
                return
            slot = int(args[0])
            if slot < self.SLOT_MIN or slot > self.SLOT_MAX:
                print(f"[CFG] ⚠️ /robot/move 범위 초과: {slot}")
                return
            self.selected_slot = slot
            self._emit_slot_leds(slot)
            print(f"[CFG] ✅ SLOT selected: {slot}")
        except Exception as e:
            print(f"[ERR] /robot/move 처리 실패: {e}")

    # ---------------- Companion/패널 LED 갱신 ----------------
    def _emit_speed_leds(self):
        """현재 JOG_VEL_PCT에 맞춰 프리셋 버튼 LED를 갱신한다.
           규칙: 현재값과 같은 프리셋 = 1, 나머지 = 0 (허용오차 ±0.5%)"""
        if not self.ui:
            return
        try:
            tol = 0.5
            current = float(JOG_VEL_PCT)
            for p in VEL_PRESETS:
                on = 1 if abs(current - p) <= tol else 0
                self.ui.send_message(f"/ui/vel/{p}", on)
            self.ui.send_message("/ui/vel/value", current)
            print(f"[UI] LED update: {current:.1f}% -> "
                  + ", ".join([f"{p}:{'ON' if abs(current-p)<=tol else 'off'}" for p in VEL_PRESETS]))
        except Exception as e:
            print("[WRN] UI LED 갱신 실패:", e)

    def _emit_speed_leds_off(self):
        """종료 시 모든 속도 LED OFF (모드 0)"""
        if not self.ui:
            return
        try:
            for p in VEL_PRESETS:
                self.ui.send_message(f"/ui/vel/{p}", 0)
            self.ui.send_message("/ui/vel/value", 0.0)
            print("[UI] LED all OFF")
        except Exception as e:
            print("[WRN] UI LED OFF 송신 실패:", e)

    # ---------------- 실행/종료 ----------------
    def serve(self):
        print("[DBG] serve_forever() 진입 예정")
        try:
            print("[RUN] Press Ctrl+C to stop.")
            self.server.serve_forever()
        except KeyboardInterrupt:
            print("\n[STOP] Keyboard interrupt")
        except Exception as e:
            print("[ERR] serve_forever 예외:", e)
            traceback.print_exc()
        finally:
            self._shutdown()

    def _shutdown(self):
        print("[DBG] _shutdown() 진입")
        # 우선 LED OFF
        self._emit_speed_leds_off()
        self._emit_slot_leds_off()
        # 로봇 정리
        try:
            self.robot.StopJOG(JOG_REF_STOP)
        except Exception as e:
            print("[WRN] StopJOG 중 예외:", e)
        try:
            self.robot.RobotEnable(0)
            self.robot.CloseRPC()
        except Exception as e:
            print("[WRN] CloseRPC 중 예외:", e)
        print("[FR5] RPC closed. Bye.")

# ---------------- 엔트리포인트 ----------------
if __name__ == "__main__":
    print("[DBG] 엔트리포인트 시작")
    srv = FR5JogServer()
    print("[DBG] FR5JogServer 인스턴스 생성 완료, serve() 호출")
    srv.serve()
