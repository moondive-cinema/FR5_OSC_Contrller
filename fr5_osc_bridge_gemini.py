#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 OSC Bridge - Combined (Jog + Preset Listener)

[병합된 기능]
1.  조그 컨트롤러 (Jog Controller)
    - 수신: /fr5/jog/{axis}, /fr5/jog/vel/... (Port 9001)
    - 발신: /ui/vel/... (Port 9003)
    - 제어: StartJOG, StopJOG

2.  프리셋 리스너 (Preset Listener)
    - 수신: /robot/move (Port 9001)
    - 발신 (Aximmetry): /vp/..., /r/... (Port 7000)
    - 발신 (UI): /ui/slot/... (Port 9003)
    - 제어: MoveCart
    - 기능: 10Hz 텔레메트리, 상태 피드백, 홈 복귀

[참고]
- 사용자 요청에 따라, 조그 이동과 프리셋 이동 간의 상태 관리는
  연동되지 않고 각자의 잠금장치(Lock)를 사용합니다.
- (조그: self.jog_lock, 프리셋: self.preset_lock)
- 두 동작을 동시에 트리거하지 않도록 사용자(운용자) 레벨에서 주의해야 합니다.
"""

import os, sys, json, time, threading, traceback
from functools import partial

# ---------------- Fairino SDK 경로 ----------------
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         'fairino-python-sdk-main', 'windows', 'fairino'))
if SDK_ROOT not in sys.path: # 경로 중복 추가 방지
    sys.path.insert(0, SDK_ROOT)
try:
    from Robot import RPC as FRRobot
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

# ---------------- 설정: 네트워크 ---------------------------
ROBOT_IP        = "192.168.2.57"

# OSC 수신 (이 스크립트가 듣는 포트 - 공통)
OSC_LISTEN_IP   = "0.0.0.0"
OSC_LISTEN_PORT = 9001

# OSC 발신 1: Aximmetry (프리셋 리스너용)
AXIM_IP         = "192.168.2.50"
AXIM_PORT       = 7000

# OSC 발신 2: UI/Companion Feedback (조그 속도 + 프리셋 슬롯 LED)
OSC_FEEDBACK_IP   = "127.0.0.1"   # Companion 이 실행되는 PC의 IP
OSC_FEEDBACK_PORT = 9003          # Companion 피드백 수신 포트

# ---------------- 설정: 프리셋 (Script 2) ----------------
DB_FILE             = "fr5_presets.json"
MOVE_VEL_PERCENT    = 50.0 # 프리셋 이동 속도
SLOTS = {
    0: "home",
    1: "cam1", 2: "cam2", 3: "cam3", 4: "cam4",
    5: "cam5", 6: "cam6", 7: "cam7", 8: "cam8", 9: "cam9"
}
# 알람 코드 (Script 2)
ALARM_OK        = 0
ALARM_ESTOP     = 1
ALARM_COLLISION = 2
ALARM_PROG_STOP = 3
ALARM_COMM_LOST = 4
ALARM_MOTIONFAIL= 5
ALARM_BUSY      = 6

# ---------------- 설정: 조그 (Script 1) ------------------
JOG_REF_MOVE = 2   # base=2
JOG_REF_STOP = 3   # base=3
JOG_MAX_DIS  = 250.0
JOG_VEL_PCT  = 40.0 # 시작 기본 속도
JOG_ACC_PCT  = 40.0
VEL_PRESETS  = (10, 20, 40, 60, 95)
AXIS_NB = {
    "x":1, "y":2, "z":3, "rx":4, "ry":5, "rz":6,
    "yaw":6
}

# ---------------------------------------------------------
# ------------------ 메인 브릿지 클래스 -------------------
# ---------------------------------------------------------

class FR5OSCBridge:
    def __init__(self):
        print("[DBG] 브릿지 서버 초기화 시작...")
        
        # --- 공통 리소스 ---
        self.robot = None
        self.server = None
        self.disp = dispatcher.Dispatcher()
        self.stop_flag = False # 텔레메트리 스레드 종료용
        self.telemetry_thread = None
        self.TOOL_IDX = 1 # 기본값
        self.USER_IDX = 0 # 기본값

        # --- OSC 클라이언트 ---
        try:
            self.axim_client = udp_client.SimpleUDPClient(AXIM_IP, AXIM_PORT)
            print(f"[DBG] Aximmetry 클라이언트 생성 -> {AXIM_IP}:{AXIM_PORT}")
        except Exception as e:
            self.axim_client = None
            print(f"[WRN] Aximmetry 클라이언트 생성 실패: {e}")

        try:
            self.ui_client = udp_client.SimpleUDPClient(OSC_FEEDBACK_IP, OSC_FEEDBACK_PORT)
            print(f"[DBG] UI Feedback 클라이언트 생성 -> {OSC_FEEDBACK_IP}:{OSC_FEEDBACK_PORT}")
        except Exception as e:
            self.ui_client = None
            print(f"[WRN] UI Feedback 클라이언트 생성 실패: {e}")

        # --- 상태 변수 (요청에 따라 분리) ---
        # 1. 조그(Jog)용 상태
        self.jog_lock = threading.Lock()
        self.is_moving_jog = False
        self.jog_vel_pct = JOG_VEL_PCT

        # 2. 프리셋(Preset)용 상태
        self.preset_lock = threading.Lock()
        self.is_moving_preset = False
        self.current_slot = 0 # 초기 슬롯은 0 (home)으로 가정
        self.target_slot_ui = 0
        self.seq_counter = 0
        self.last_alarm_code = None
        self.arrived_pulse_thread = None
        self.arrived_pulse_cancel = threading.Event()
        self.db_data = self._load_db(DB_FILE)

        # --- 초기화 실행 ---
        self._init_robot() # 로봇 연결 및 설정
        self._init_osc()   # OSC 경로 매핑 및 서버 바인딩
        
        # 텔레메트리 스레드 시작 (프리셋 리스너 기능)
        self._init_telemetry_thread() 

        # 초기 LED 상태 전송
        self._emit_slot_leds(self.current_slot) # 프리셋 슬롯 LED
        self._emit_speed_leds()                 # 조그 속도 LED
        
        print("[DBG] 브릿지 서버 초기화 완료.")

    # -----------------------------------------------------
    # --- 1. 초기화 및 종료 (공통) ---
    # -----------------------------------------------------

    def _init_robot(self):
        """로봇 연결 및 두 스크립트의 초기화 절차 통합"""
        print(f"[DBG] 🔌 FR5 연결 시도 → {ROBOT_IP}")
        try:
            self.robot = FRRobot(ROBOT_IP)
            print("[DBG] 🤖 FR5 연결 객체 생성 완료:", self.robot)
        except Exception as e:
            print("[ERR] 🛑 FRRobot 연결 실패:", e)
            traceback.print_exc()
            sys.exit(1)

        try:
            print("[DBG] RobotEnable(1) 호출")
            self.robot.RobotEnable(1)
            print("[DBG] Mode(0) 호출 (자동)")
            self.robot.Mode(0)
            print("[DBG] DragTeachSwitch(0) 호출")
            self.robot.DragTeachSwitch(0)
            
            # (조그 컨트롤러의 전역 속도 설정)
            print("[DBG] SetSpeed(30.0) 호출") 
            rc = self.robot.SetSpeed(30.0)
            print(f"[DBG] SetSpeed 리턴값: {rc}")

            time.sleep(1) # 안정화
            print("[DBG] ✅ 로봇 자동 모드 활성화 완료")
            
            # (프리셋 리스너의 좌표계 읽기)
            self.TOOL_IDX = int(getattr(self.robot.robot_state_pkg, "tool", 1))
            self.USER_IDX = int(getattr(self.robot.robot_state_pkg, "user", 0))
            print(f"[DBG] ℹ️ 현재 좌표계: Tool={self.TOOL_IDX}, User={self.USER_IDX}")

        except Exception as e:
            print("[ERR] ⚠️ 로봇 초기화 호출 중 예외:", e)
            traceback.print_exc()

    def _init_osc(self):
        """두 스크립트의 모든 OSC 경로를 단일 디스패처에 등록"""
        print("[DBG] OSC Dispatcher 구성 시작")

        # --- 스크립트 1 (조그) 경로 등록 ---
        for axis in AXIS_NB.keys():
            path = f"/fr5/jog/{axis}"
            # functools.partial을 사용하여 핸들러에 axis 인수 고정
            self.disp.map(path, partial(self._cb_jog, axis=axis))
            print(f"[DBG] map() 등록: {path} -> _cb_jog(axis='{axis}')")

        for preset in VEL_PRESETS:
            p = f"/fr5/jog/vel/{preset}"
            self.disp.map(p, partial(self._cb_vel_preset, preset_val=preset))
            print(f"[DBG] map() 등록: {p} -> set vel {preset}%")

        self.disp.map("/fr5/jog/vel", self._cb_vel_arg)
        print("[DBG] map() 등록: /fr5/jog/vel -> _cb_vel_arg <number>")
        
        self.disp.map("/fr5/jog/vel/get", self._get_vel)
        print("[DBG] map() 등록: /fr5/jog/vel/get -> _get_vel")

        # --- 스크립트 2 (프리셋) 경로 등록 ---
        self.disp.map("/robot/move", self._cb_move_slot)
        print("[DBG] map() 등록: /robot/move -> _cb_move_slot")
        
        self.disp.map("/ping", lambda a,*b: print("✅ /ping 수신"))
        print("[DBG] map() 등록: /ping")

        # 공통 기본 핸들러
        def _default_log(addr, *args):
            print(f"[RX] 알 수 없는 주소: {addr} {args}")
        self.disp.set_default_handler(_default_log)

        # --- OSC 서버 바인딩 ---
        print(f"[DBG] OSC 서버 바인딩 시도: {OSC_LISTEN_IP}:{OSC_LISTEN_PORT}")
        try:
            self.server = osc_server.ThreadingOSCUDPServer(
                (OSC_LISTEN_IP, OSC_LISTEN_PORT), self.disp
            )
            print(f"[DBG] ✅ OSC 서버 리슨 중: {self.server.server_address}")
        except Exception as e:
            print("[ERR] 🛑 OSC 서버 바인딩 실패:", e)
            traceback.print_exc()
            sys.exit(1)

    def serve(self):
        """메인 실행 루프"""
        print(f"\n--- 🤖 FR5 OSC 통합 브릿지 시작 ---")
        print(f"  수신 대기: {self.server.server_address[0]}:{self.server.server_address[1]}")
        print(f"  로봇 IP: {ROBOT_IP} (T:{self.TOOL_IDX}, U:{self.USER_IDX})")
        print(f"  Aximmetry: {AXIM_IP}:{AXIM_PORT}")
        print(f"  UI 피드백: {OSC_FEEDBACK_IP}:{OSC_FEEDBACK_PORT}")
        print(f"  [프리셋 명령]")
        print(f"    /robot/move [0~9]")
        print(f"  [조그 명령]")
        print(f"    /fr5/jog/x, y, z, rx, ry, rz [1.0 | 0.0]")
        print(f"    /fr5/jog/vel/{{10|20|...}} [1.0]")
        print("  종료: Ctrl+C\n")
        
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            print("\n[STOP] Keyboard interrupt 감지")
        except Exception as e:
            print("[ERR] serve_forever 예외:", e)
            traceback.print_exc()
        finally:
            self._shutdown()

    def _shutdown(self):
        """두 스크립트의 종료 절차 통합"""
        print("\n[DBG] _shutdown() 진입. 모든 기능 종료 중...")
        self.stop_flag = True # 텔레메트리 스레드 중지 신호

        # 텔레메트리 스레드 종료 대기
        if self.telemetry_thread:
            print("  ➡️ 텔레메트리 스레드 종료 대기...")
            self.telemetry_thread.join(timeout=0.5)

        # 도착 펄스 스레드 중지 (프리셋)
        if self.arrived_pulse_thread and self.arrived_pulse_thread.is_alive():
            self.arrived_pulse_cancel.set()
            self.arrived_pulse_thread.join(timeout=0.2)

        # 모든 LED 끄기
        print("  ➡️ 모든 UI LED 끄는 중...")
        self._emit_speed_leds_off() # 조그 속도 LED
        self._emit_slot_leds_off()  # 프리셋 슬롯 LED

        # 로봇 정리
        try:
            print("  ➡️ StopJOG 호출 (조그 비상 정지)")
            self.robot.StopJOG(JOG_REF_STOP)
        except Exception as e:
            print(f"  [WRN] StopJOG 중 예외: {e}")

        # 홈 복귀 시도 (프리셋)
        home_pose = self.db_data.get("home", {}).get("val")
        if isinstance(home_pose, list) and len(home_pose) >= 6:
            if not self.preset_lock.locked() and not self.jog_lock.locked():
                print("  ➡️ 홈 위치로 복귀 시도...")
                try:
                    self.robot.MoveCart(desc_pos=home_pose, tool=self.TOOL_IDX, user=self.USER_IDX, vel=MOVE_VEL_PERCENT)
                    print("  [DBG] 홈 위치 이동 완료.")
                    self.current_slot = 0
                except Exception as e:
                    print(f"  [WRN] 홈 위치 이동 중 예외: {e}")
            else:
                print("  [WRN] 로봇이 다른 작업으로 잠겨있어 홈 복귀 생략.")
        else:
            print(f"  [WRN] 'home' 좌표({DB_FILE})가 없어 홈 복귀 생략.")

        # OSC 서버 종료
        try:
            print("  ➡️ OSC 서버 종료 중...")
            self.server.shutdown()
            self.server.server_close()
        except Exception as e:
            print(f"  [WRN] OSC 서버 종료 중 예외: {e}")

        # 로봇 연결 종료
        try:
            print("  ➡️ 로봇 연결 해제 (Enable(0), CloseRPC)")
            self.robot.RobotEnable(0)
            self.robot.CloseRPC()
        except Exception as e:
            print(f"  [WRN] CloseRPC 중 예외: {e}")
            
        print("[FR5] RPC closed. Bye.")
        sys.exit(0)

    # -----------------------------------------------------
    # --- 2. 조그 컨트롤러 (Script 1) 기능 ---
    # -----------------------------------------------------

    def _cb_jog(self, addr, val, axis):
        """조그 버튼 눌림/뗌 처리"""
        # print(f"[DBG] → _cb_jog: axis={axis}, val={val}")
        try:
            val = float(val)
        except Exception as e:
            print("[ERR] _cb_jog: val float 변환 실패:", e); return

        if axis not in AXIS_NB:
            print(f"[WRN] 미지원 축: {axis}"); return

        try:
            if abs(val) < 1e-6:
                # --- STOP JOG ---
                if not self.is_moving_jog: # 이미 정지 상태면 무시
                    return
                
                print(f"[DBG] Stop 요청 → StopJOG({JOG_REF_STOP})")
                rc = self.robot.StopJOG(JOG_REF_STOP)
                # print(f"[DBG] StopJOG 리턴값: {rc}")
                print(f"[JOG] {axis:<3} STOP")
                self.is_moving_jog = False
                if self.jog_lock.locked():
                    self.jog_lock.release() # 조그 잠금 해제
                return

            # --- START JOG ---
            # 사용자 요청: 상태 관리 연동 안 함. 조그 자체 락만 사용.
            if not self.jog_lock.acquire(blocking=False):
                print(f"[JOG] 🚫 조그 시작 거부 (BUSY, 이미 다른 축 조그 중)")
                self._send_alarm(ALARM_BUSY, "BUSY (Jog Active)")
                return
            
            self.is_moving_jog = True # 잠금 성공, 이동 시작
            nb  = AXIS_NB[axis]
            dir = 1 if val > 0 else 0  # 0=음, 1=양
            
            # print(f"[DBG] Start 요청 → StartJOG(...)")
            rc = self.robot.StartJOG(JOG_REF_MOVE, nb, dir, JOG_MAX_DIS,
                                     self.jog_vel_pct, JOG_ACC_PCT)
            
            # print(f"[DBG] StartJOG 리턴값: {rc}")
            if rc != 0:
                print(f"[ERR] 🛑 StartJOG 실패: err={rc}")
                self.is_moving_jog = False # 실패 시 상태 복원
                if self.jog_lock.locked():
                    self.jog_lock.release() # 잠금 해제
                return

            print(f"[JOG] {axis:<3} {'+' if dir else '-'} vel={self.jog_vel_pct}%")

        except Exception as e:
            print(f"[ERR] ⚠️ _cb_jog 실행 중 예외: {e}")
            traceback.print_exc()
            self.is_moving_jog = False # 예외 발생 시 잠금 해제
            if self.jog_lock.locked():
                self.jog_lock.release()

    def _cb_vel_preset(self, addr, *args, preset_val=None):
        """조그 속도 프리셋 버튼 처리"""
        if preset_val is not None:
            self._set_jog_vel_pct(float(preset_val))

    def _cb_vel_arg(self, addr, *args):
        """/fr5/jog/vel <number> 형태 처리"""
        if not args:
            print("[CFG] ⚠️ /fr5/jog/vel 인자 없음"); return
        try:
            val = float(args[0])
            self._set_jog_vel_pct(val)
        except Exception as e:
            print(f"[CFG] ⚠️ /fr5/jog/vel 인자 변환 실패: {args} ({e})")

    def _set_jog_vel_pct(self, pct: float):
        """조그 속도 설정 (조그 이동 중이 아닐 때만)"""
        if self.is_moving_jog: # 조그 자체 플래그 사용
            print(f"[CFG] 🚫 조그 속도 변경 무시 (조그 이동 중) 요청={pct}%")
            return
        
        old = self.jog_vel_pct
        v = max(0.0, min(100.0, float(pct)))
        self.jog_vel_pct = v
        print(f"[CFG] ✅ JOG_VEL_PCT: {old:.1f}% -> {self.jog_vel_pct:.1f}%")
        
        self._emit_speed_leds() # UI LED 갱신

    def _get_vel(self, addr, *args):
        """현재 조그 속도 값 전송"""
        print(f"[CFG] ℹ️ JOG_VEL_PCT = {self.jog_vel_pct:.1f}%")
        if self.ui_client:
            self.ui_client.send_message("/ui/vel/value", float(self.jog_vel_pct))

    def _emit_speed_leds(self):
        """조그 속도 프리셋 버튼 LED 갱신"""
        if not self.ui_client: return
        try:
            current = float(self.jog_vel_pct)
            for p in VEL_PRESETS:
                on = 1 if abs(current - p) <= 0.5 else 0
                self.ui_client.send_message(f"/ui/vel/{p}", on)
            self.ui_client.send_message("/ui/vel/value", current)
        except Exception as e:
            print("[WRN] UI Speed LED 갱신 실패:", e)

    def _emit_speed_leds_off(self):
        """종료 시 모든 조그 속도 LED OFF"""
        if not self.ui_client: return
        try:
            for p in VEL_PRESETS:
                self.ui_client.send_message(f"/ui/vel/{p}", 0)
            self.ui_client.send_message("/ui/vel/value", 0.0)
            print("[UI] Speed LED all OFF")
        except Exception as e:
            print("[WRN] UI Speed LED OFF 송신 실패:", e)

    # -----------------------------------------------------
    # --- 3. 프리셋 리스너 (Script 2) 기능 ---
    # -----------------------------------------------------

    def _load_db(self, path):
        """프리셋 JSON 파일 로드"""
        if not os.path.exists(path):
            print(f"⚠️ DB 없음: {path}")
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"⚠️ DB 로드 실패: {e}")
            return {}

    def _cb_move_slot(self, address, *args):
        """프리셋 슬롯 이동 명령 처리"""
        if not args: print("⚠️ /robot/move 인자 없음 (0~9 필요)"); return
        try: target_slot = int(args[0])
        except ValueError: print(f"⚠️ 잘못된 슬롯 인자: {args[0]}"); return
        if target_slot not in SLOTS: print(f"⚠️ 유효하지 않은 슬롯: {target_slot}"); return

        slot_name = SLOTS[target_slot]
        if not self.db_data or slot_name not in self.db_data: 
            print(f"⚠️ '{slot_name}' 좌표 없음 ({DB_FILE})"); return
        target_pose = self.db_data[slot_name].get("val")
        if not isinstance(target_pose, list) or len(target_pose) < 6: 
            print(f"⚠️ '{slot_name}' 좌표값 불완전"); return

        # [중요] 프리셋 자체 잠금 확인 (사용자 요청: 조그 락과 연동 안 함)
        if not self.preset_lock.acquire(blocking=False):
            print("🟡 프리셋 이동 거절: BUSY (이미 다른 프리셋 이동 중)")
            self._send_alarm(ALARM_BUSY, "BUSY (Preset Move Active)")
            return

        try:
            print(f"🚚 프리셋 이동 시작 → '{slot_name}' (슬롯 {target_slot})")
            self._emit_slot_leds(target_slot) # 슬롯 LED 업데이트

            if self.arrived_pulse_thread and self.arrived_pulse_thread.is_alive():
                self.arrived_pulse_cancel.set()
                self.arrived_pulse_thread.join(timeout=0.3)

            self.is_moving_preset = True # 프리셋 이동 플래그
            self.target_slot_ui = target_slot
            self._send_robot_operation(self.current_slot, self.target_slot_ui, 1, 0) # moving=1

            # MoveCart는 블로킹(blendT=-1.0)으로 호출됨 (SDK 3.9)
            err = self.robot.MoveCart(
                desc_pos=target_pose, 
                tool=self.TOOL_IDX, 
                user=self.USER_IDX, 
                vel=MOVE_VEL_PERCENT
            )
            if err is None: err = 0

            if err != 0:
                print(f"🛑 이동 명령 실패 (MoveCart 코드 {err})")
                self.is_moving_preset = False; self.target_slot_ui = self.current_slot
                self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)
                self._send_alarm(ALARM_MOTIONFAIL, f"MOTION_FAIL(code={err})")
                self.preset_lock.release() # 잠금 해제
                return

            # MoveCart가 블로킹이므로, 이 시점은 도착 후임.
            ok, reason = self._wait_until_arrived(robot=self.robot, timeout_s=120.0)

            if ok:
                print(f"✅ 도착 확인 → '{slot_name}'")
                self.current_slot   = target_slot
                self.is_moving_preset = False
                self.target_slot_ui = self.current_slot
                self._pulse_arrived_async(self.current_slot, duration=1.0, hz=20)
                # 펄스 시작 후 기본 상태(moving=0, arrived=0) 즉시 전송
                self._send_robot_operation(self.current_slot, self.target_slot_ui, 0, 0)
            else:
                print(f"🛑 도착 실패 (wait): {reason}")
                self.is_moving_preset = False; self.target_slot_ui = self.current_slot
                self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)
                alarm_map = {
                    "COMM_LOST": (ALARM_COMM_LOST, "COMM_LOST"),
                    "E_STOP": (ALARM_ESTOP, "E_STOP"),
                    "COLLISION": (ALARM_COLLISION, "COLLISION"),
                    "TIMEOUT": (ALARM_MOTIONFAIL, "MOTION_TIMEOUT")
                }
                code, msg = alarm_map.get(reason, (ALARM_PROG_STOP, reason))
                self._send_alarm(code, msg)

        except Exception as e:
            print(f"🛑 _cb_move_slot 처리 중 예외: {e}")
            traceback.print_exc()
            self.is_moving_preset = False; self.target_slot_ui = self.current_slot
            self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)
        finally:
            if self.preset_lock.locked():
                self.preset_lock.release() # 작업 완료 후 잠금 해제

    def _emit_slot_leds(self, selected:int):
        """프리셋 슬롯 버튼 LED 갱신"""
        if not self.ui_client: return
        try:
            keys = sorted(SLOTS.keys())
            for k in keys:
                self.ui_client.send_message(f"/ui/slot/{k}", 1 if k == int(selected) else 0)
            self.ui_client.send_message("/ui/slot/value", int(selected))
            # print(f"[UI] SLOT LED update: selected={selected}")
        except Exception as e:
            print(f"[WRN] UI Slot LED 갱신 실패: {e}")

    def _emit_slot_leds_off(self):
        """종료 시 모든 프리셋 슬롯 LED OFF"""
        if not self.ui_client: return
        try:
            keys = sorted(SLOTS.keys())
            for k in keys:
                self.ui_client.send_message(f"/ui/slot/{k}", 0)
            self.ui_client.send_message("/ui/slot/value", -1)
            print("[UI] Slot LED all OFF")
        except Exception as e:
            print("[WRN] UI Slot LED OFF 송신 실패:", e)

    # -----------------------------------------------------
    # --- 4. 텔레메트리 및 피드백 (Script 2) ---
    # -----------------------------------------------------

    def _init_telemetry_thread(self):
        """텔레메트리 루프를 별도 스레드에서 시작"""
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()
        print("[DBG] 텔레메트리 스레드 시작됨.")

    def _telemetry_loop(self):
        """Aximmetry 피드백 및 상태 모니터링 루프"""
        t_pose = t_hb = 0.0
        prev_estop = None
        prev_collision = None

        if not hasattr(self, "_last_comm_ok"): self._last_comm_ok = None
        if not hasattr(self, "_last_system_state"): self._last_system_state = None
        if not hasattr(self, "_last_hb_interval"): self._last_hb_interval = None
        
        while self.robot is None and not self.stop_flag:
            time.sleep(0.1)
        
        print("[DBG] 텔레메트리 루프 정상 가동.")
        while not self.stop_flag:
            now = time.time()
            
            # Aximmetry 클라이언트가 없으면 텔레메트리/상태 전송 건너뜀
            if self.axim_client:
                # --- 10Hz 텔레메트리 ---
                if now - t_pose >= 0.1: 
                    try:
                        q = [float(self.robot.robot_state_pkg.jt_cur_pos[i]) for i in range(6)]
                        tcp = [float(self.robot.robot_state_pkg.tl_cur_pos[i]) for i in range(6)]
                        tcp_speed = [
                            float(self.robot.robot_state_pkg.actual_TCP_CmpSpeed[0]),
                            float(self.robot.robot_state_pkg.actual_TCP_CmpSpeed[1]),
                        ]
                        self.axim_client.send_message("/r/joint", q)
                        self.axim_client.send_message("/r/tcp", tcp)
                        self.axim_client.send_message("/r/tcp_speed", tcp_speed)
                    except Exception:
                        pass 
                    t_pose = now

                # --- 상태 평가 및 HB ---
                comm_ok = self._probe_comm_ok(self.robot)
                if self._last_comm_ok is None: self._last_comm_ok = comm_ok
                elif self._last_comm_ok != comm_ok:
                    print("🛑 텔레메트리: 통신 오류" if comm_ok == 0 else "✅ 텔레메트리: 통신 복구")
                    self._last_comm_ok = comm_ok

                power_on     = int(getattr(self.robot.robot_state_pkg, "rbtEnableState", 0)) if comm_ok else 0
                estop        = int(getattr(self.robot.robot_state_pkg, "EmergencyStop", 0))   if comm_ok else 0
                collision    = int(getattr(self.robot.robot_state_pkg, "collisionState", 0))  if comm_ok else 0
                main_code    = int(getattr(self.robot.robot_state_pkg, "main_code", 0))       if comm_ok else 0

                if comm_ok == 0 or estop == 1: system_state = 2 # 셧다운
                elif collision == 1 or main_code != 0: system_state = 1 # 경고
                else: system_state = 0 # 정상

                # 이동 중(is_moving_preset 또는 is_moving_jog)이면 0.5초, 아니면 1.0초
                is_any_moving = self.is_moving_preset or self.is_moving_jog
                hb_interval = 5.0 if system_state in (1, 2) else (0.5 if is_any_moving else 1.0)

                new_alarm = None
                if comm_ok == 0: new_alarm = (ALARM_COMM_LOST, "COMM_LOST")
                elif estop == 1: new_alarm = (ALARM_ESTOP, "E_STOP")
                elif collision == 1: new_alarm = (ALARM_COLLISION, "COLLISION")
                elif main_code != 0: new_alarm = (ALARM_PROG_STOP, f"PROGRAM_STOP(main_code={main_code})")
                else: new_alarm = (ALARM_OK, "OK")

                if new_alarm and (new_alarm[0] != self.last_alarm_code):
                    self._send_alarm(new_alarm[0], new_alarm[1])

                send_immediately = (self._last_system_state != system_state)
                if send_immediately:
                    self.seq_counter = (self.seq_counter + 1) & 0x7fffffff
                    self._send_robot_status(system_state, self.seq_counter)
                    self._last_system_state = system_state
                    t_hb = now

                if now - t_hb >= hb_interval:
                    self.seq_counter = (self.seq_counter + 1) & 0x7fffffff
                    self._send_robot_status(system_state, self.seq_counter)
                    self._last_hb_interval = hb_interval
                    t_hb = now

                if prev_estop is None: prev_estop, prev_collision = estop, collision
                else:
                    if prev_estop != estop: print("🛑 E-STOP ON" if estop else "✅ E-STOP OFF"); prev_estop = estop
                    if prev_collision != collision: print("⚠️ COLLISION ON" if collision else "✅ COLLISION OFF"); prev_collision = collision
            
            else: # Aximmetry 클라이언트가 없을 때
                if not self._probe_comm_ok(self.robot):
                    print("🛑 텔레메트리: 통신 오류 (Axim 전송 비활성화)")

            time.sleep(0.01) # 루프 최소 간격

    def _probe_comm_ok(self, robot):
        try: _ = robot.robot_state_pkg.second; return 1
        except Exception: return 0

    def _send_robot_operation(self, cur:int, tgt:int, moving:int, arrived:int):
        if not self.axim_client: return
        try: self.axim_client.send_message("/vp/robot_operation", [int(cur), int(tgt), int(moving), int(arrived)])
        except Exception as e: print(f"⚠️ robot_operation 전송 실패: {e}")

    def _send_robot_status(self, system_state:int, seq:int):
        if not self.axim_client: return
        try: self.axim_client.send_message("/vp/robot_status", [int(system_state), int(seq)])
        except Exception as e: print(f"⚠️ robot_status 전송 실패: {e}")

    def _send_alarm(self, code:int, message:str):
        if not self.axim_client: return
        if code == self.last_alarm_code and code != ALARM_OK:
             return
        try:
            self.axim_client.send_message("/vp/robot_alarm", [int(code), str(message)])
            self.last_alarm_code = code
        except Exception as e: print(f"⚠️ robot_alarm 전송 실패: {e}")

    def _pulse_arrived_worker(self, slot:int, duration:float, hz:int):
        """도착 펄스 전송 스레드 워커"""
        interval = 1.0 / float(hz)
        t_end = time.time() + max(0.05, duration)
        try:
            while time.time() < t_end:
                if self.arrived_pulse_cancel.is_set(): return
                self._send_robot_operation(slot, slot, 0, 1) # arrived=1
                time.sleep(interval)
            self._send_robot_operation(slot, slot, 0, 0) # 마지막 0 프레임
        except Exception as e:
            print(f"🛑 _pulse_arrived_worker 예외: {e}")
            try: self._send_robot_operation(slot, slot, 0, 0)
            except: pass

    def _pulse_arrived_async(self, slot:int, duration:float=1.0, hz:int=20):
        """도착 펄스 전송 스레드 시작"""
        if self.arrived_pulse_thread and self.arrived_pulse_thread.is_alive():
            self.arrived_pulse_cancel.set()
            self.arrived_pulse_thread.join(timeout=0.3)
        self.arrived_pulse_cancel.clear()
        self.arrived_pulse_thread = threading.Thread(
            target=self._pulse_arrived_worker, args=(slot, duration, hz), daemon=True
        )
        self.arrived_pulse_thread.start()

    def _wait_until_arrived(self, robot, poll_interval=0.05, timeout_s=120.0):
        """SDK 1.1의 motion_done을 기반으로 도착 대기"""
        start = time.time()
        while (time.time() - start) < timeout_s:
            if self.stop_flag: return (False, "SHUTDOWN")
            try: _ = robot.robot_state_pkg.second
            except Exception: return (False, "COMM_LOST")

            estop     = int(getattr(robot.robot_state_pkg, "EmergencyStop", 0))
            collision = int(getattr(robot.robot_state_pkg, "collisionState", 0))
            main_code = int(getattr(robot.robot_state_pkg, "main_code", 0))
            motion_dn = int(getattr(robot.robot_state_pkg, "motion_done", 0))

            if estop == 1:      return (False, "E_STOP")
            if collision == 1:  return (False, "COLLISION")
            if main_code != 0:  return (False, f"MAIN_CODE_{main_code}")
            if motion_dn == 1:  return (True, "OK") # 도착!

            time.sleep(poll_interval)
        return (False, "TIMEOUT") # 타임아웃


# -----------------------------------------------------
# --- 엔트리포인트 ---
# -----------------------------------------------------
if __name__ == "__main__":
    srv = FR5OSCBridge()
    srv.serve()