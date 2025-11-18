#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 OSC Bridge - v04 (Hybrid: Joint-Based + Jog + Preset Listener + Swap)

[v04 하이브리드 변경 사항]
- 이 버전은 프리셋 이동 시 'joint_val' (관절 값)을 우선 사용합니다.
- 'joint_val'이 있으면: MoveJ (관절 이동)를 사용하여 360도 플립 문제를 해결합니다.
- 'joint_val'이 없으면: 기존 'val' (Cartesian)을 사용하여 MoveCart로 이동합니다 (하위 호환).
- 프리셋 저장/스왑 시 'val'과 'joint_val' 값을 *모두* 저장하여 DB를 업그레이드합니다.

[기능]
1.  조그 컨트롤러 (Jog Controller)
    - 수신: /fr5/jog/{axis}, /fr5/jog/vel/... (Port 9001)
    - 제어: StartJOG, StopJOG

2.  프리셋 리스너 (Preset Listener)
    - 수신: /robot/move (Port 9001)
    - 제어: [V04] MoveJ (joint_val 우선) 또는 MoveCart (val 폴백)
    - 기능: 10Hz 텔레메트리, 상태 피드백, 홈 복귀

3.  2단계 스왑 (Swap)
    - 1단계 수신: /robot/record/temp (Port 9001)
      - 현재 위치(A)의 [joint_val, val]을 'temp_pose'에 10초간 임시 저장
    - 2단계 수신: /robot/move [슬롯N] (Port 9001)
      - 10초 내에 이 메시지가 오면, 슬롯N의 기존 좌표(B)와 'temp_pose'(A)를 스왑.
"""

import os, sys, json, time, threading, traceback
from functools import partial

# ---------------- Fairino SDK 경로 ----------------
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         'fairino-python-sdk-main', 'linux', 'fairino'))
if SDK_ROOT not in sys.path: # 경로 중복 추가 방지
    sys.path.insert(0, SDK_ROOT)
try:
    import Robot
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
RECORD_MODE_TIMEOUT = 10.0 # 'Record-Arm' (임시 저장) 모드 활성화 시간 (초)
MOVE_VEL_PERCENT    = 40.0 # 프리셋 이동 속도
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
        
        self.robot = None
        self.server = None
        self.disp = dispatcher.Dispatcher()
        self.stop_flag = False
        self.telemetry_thread = None
        self.TOOL_IDX = 1
        self.USER_IDX = 0

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

        # --- 상태 변수 ---
        self.jog_lock = threading.Lock()
        self.is_moving_jog = False
        self.jog_vel_pct = JOG_VEL_PCT

        self.preset_lock = threading.Lock()
        self.db_lock = threading.Lock()
        self.is_moving_preset = False
        self.current_slot = 0
        self.target_slot_ui = 0
        self.seq_counter = 0
        self.last_alarm_code = None
        self.arrived_pulse_end_time = 0.0
        
        with self.db_lock:
            self.db_data = self._load_db(DB_FILE)

        # [V04 변경] 임시 저장소 (dict)
        self.temp_pose = None
        self.temp_pose_timestamp = 0.0

        # --- 초기화 실행 ---
        self._init_robot() 
        
        # [V04 신규] 프리셋 포맷 검사
        self._check_preset_format()
        
        self._init_osc()
        self._init_telemetry_thread() 

        self._emit_slot_leds(self.current_slot)
        self._emit_speed_leds()
        
        print("[DBG] 브릿지 서버 초기화 완료.")

    # -----------------------------------------------------
    # --- 1. 초기화 및 종료 (공통) ---
    # -----------------------------------------------------

    def _init_robot(self):
        """로봇 연결 및 초기화"""
        print(f"[DBG] 🔌 FR5 연결 시도 → {ROBOT_IP}")
        try:
            self.robot = Robot.RPC(ROBOT_IP)
            print("[DBG] 🤖 FR5 연결 객체 생성 완료:", self.robot)
        except Exception as e:
            print("[ERR] 🛑 Robot 연결 실패:", e)
            traceback.print_exc()
            sys.exit(1)
        
        try:
            print("[DBG] ResetAllError() 호출 (기존 오류 클리어)")
            self.robot.ResetAllError()
            time.sleep(0.5)

            print("[DBG] RobotEnable(1) 호출")
            self.robot.RobotEnable(1)
            print("[DBG] Mode(0) 호출 (자동)")
            self.robot.Mode(0)
            print("[DBG] DragTeachSwitch(0) 호출")
            self.robot.DragTeachSwitch(0)
            print("[DBG] SetSpeed(30.0) 호출") 
            self.robot.SetSpeed(30.0)

            time.sleep(1) # 안정화
            print("[DBG] ✅ 로봇 자동 모드 활성화 완료")
            
            self.TOOL_IDX = int(getattr(self.robot.robot_state_pkg, "tool", 1))
            self.USER_IDX = int(getattr(self.robot.robot_state_pkg, "user", 0))
            print(f"[DBG] ℹ️ 현재 좌표계: Tool={self.TOOL_IDX}, User={self.USER_IDX}")

        except Exception as e:
            print("[ERR] ⚠️ 로봇 초기화 호출 중 예외:", e)
            traceback.print_exc()

    def _check_preset_format(self):
        """v04 호환성 검사. 'joint_val'이 없는 경우 경고만 표시 (마이그레이션 권장)"""
        if not self.robot: return
        
        with self.db_lock:
            if not self.db_data or "home" not in self.db_data:
                print(f"[V04 경고] ⚠️ '{DB_FILE}' 프리셋 파일이 비어있습니다. 'Record-Arm' 기능으로 새로 생성하세요.")
                return

            home_entry = self.db_data.get("home", {})
            
            if "joint_val" not in home_entry or not home_entry.get("joint_val"):
                 print("="*60)
                 print(f"[V04 경고] ⚠️ '{DB_FILE}'에 'joint_val' (관절 값)이 없습니다.")
                 print(f"  이 스크립트(v04)는 'joint_val'이 없는 프리셋의 경우,")
                 print(f"  기존 'val' (Cartesian 값)을 사용하여 MoveCart로 이동합니다.")
                 print(f"  이 경우 360도 플립 현상이 계속 발생할 수 있습니다.")
                 print(f"  [권장] 'migrate_presets_v4.py' 스크립트를 한 번 실행하여")
                 print(f"  모든 프리셋에 'joint_val'을 추가하십시오.")
                 print("="*60)
            else:
                 print(f"[V04 확인] ✅ '{DB_FILE}'이(가) 'joint_val'을 지원하는 것을 확인했습니다.")


    def _init_osc(self):
        """OSC 경로를 단일 디스패처에 등록"""
        print("[DBG] OSC Dispatcher 구성 시작")

        for axis in AXIS_NB.keys():
            path = f"/fr5/jog/{axis}"
            self.disp.map(path, partial(self._cb_jog, axis=axis))
            print(f"[DBG] map() 등록: {path}")

        for preset in VEL_PRESETS:
            p = f"/fr5/jog/vel/{preset}"
            self.disp.map(p, partial(self._cb_vel_preset, preset_val=preset))
            print(f"[DBG] map() 등록: {p}")

        self.disp.map("/fr5/jog/vel", self._cb_vel_arg)
        print("[DBG] map() 등록: /fr5/jog/vel")
        self.disp.map("/fr5/jog/vel/get", self._get_vel)
        print("[DBG] map() 등록: /fr5/jog/vel/get")
        self.disp.map("/robot/move", self._cb_move_slot)
        print("[DBG] map() 등록: /robot/move")
        self.disp.map("/robot/record/temp", self._cb_record_temp)
        print("[DBG] map() 등록: /robot/record/temp")
        self.disp.map("/robot/terminate", self._cb_terminate)
        print("[DBG] map() 등록: /robot/terminate")
        self.disp.map("/ping", lambda a,*b: print("✅ /ping 수신"))
        print("[DBG] map() 등록: /ping")

        def _default_log(addr, *args):
            print(f"[RX] 알 수 없는 주소: {addr} {args}")
        self.disp.set_default_handler(_default_log)

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
        print(f"\n--- 🤖 FR5 OSC 통합 브릿지 시작 (v04 - Joint/Cartesian 혼합) ---")
        print(f"  [안내] 'joint_val'이 있는 프리셋은 MoveJ로, 없으면 MoveCart로 이동합니다.")
        print(f"  수신 대기: {self.server.server_address[0]}:{self.server.server_address[1]}")
        print(f"  로봇 IP: {ROBOT_IP} (T:{self.TOOL_IDX}, U:{self.USER_IDX})")
        print(f"  Aximmetry: {AXIM_IP}:{AXIM_PORT}")
        print(f"  UI 피드백: {OSC_FEEDBACK_IP}:{OSC_FEEDBACK_PORT}")
        print(f"  [프리셋 명령]")
        print(f"    /robot/move [0~9]       (이동 또는 스왑 확정)")
        print(f"    /robot/record/temp      (현재 [조인트+TCP] 위치 임시 저장/스왑 시작)")
        print(f"    /robot/terminate        (이 브릿지 스크립트 종료)")
        print(f"  [조그 명령]")
        print(f"    /fr5/jog/x, y, z, rx, ry, rz [1.0 | 0.0]")
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
        """종료 절차 통합"""
        print("\n[DBG] _shutdown() 진입. 모든 기능 종료 중...")
        self.stop_flag = True

        if self.telemetry_thread:
            print("  ➡️ 텔레메트리 스레드 종료 대기...")
            self.telemetry_thread.join(timeout=0.5)

        print("  ➡️ 모든 UI LED 끄는 중...")
        self._emit_speed_leds_off()
        self._emit_slot_leds_off()
        if self.ui_client:
            try: self.ui_client.send_message("/ui/record/armed", 0)
            except: pass

        try:
            print("  ➡️ StopJOG 호출 (조그 비상 정지)")
            self.robot.StopJOG(JOG_REF_STOP)
        except Exception as e:
            print(f"  [WRN] StopJOG 중 예외: {e}")

        # [V04 변경] 홈 복귀 시 joint_val 우선
        home_joint_pose = None
        home_cart_pose = None
        with self.db_lock:
            home_entry = self.db_data.get("home", {})
            home_joint_pose = home_entry.get("joint_val")
            home_cart_pose = home_entry.get("val")
            
        if not self.preset_lock.locked() and not self.jog_lock.locked():
            if home_joint_pose and isinstance(home_joint_pose, list) and len(home_joint_pose) >= 6:
                print("  ➡️ 홈 위치로 복귀 시도 (MoveJ 사용)...")
                try:
                    self.robot.MoveJ(joint_pos=home_joint_pose, tool=self.TOOL_IDX, user=self.USER_IDX, vel=MOVE_VEL_PERCENT)
                    print("  [DBG] 홈 위치 이동 완료.")
                    self.current_slot = 0
                except Exception as e:
                    print(f"  [WRN] 홈(MoveJ) 이동 중 예외: {e}")
            elif home_cart_pose and isinstance(home_cart_pose, list) and len(home_cart_pose) >= 6:
                print("  ➡️ 홈 위치로 복귀 시도 (MoveCart 폴백 사용)...")
                try:
                    self.robot.MoveCart(desc_pos=home_cart_pose, tool=self.TOOL_IDX, user=self.USER_IDX, vel=MOVE_VEL_PERCENT)
                    print("  [DBG] 홈 위치 이동 완료.")
                    self.current_slot = 0
                except Exception as e:
                    print(f"  [WRN] 홈(MoveCart) 이동 중 예외: {e}")
            else:
                print(f"  [WRN] 'home' 좌표({DB_FILE})가 없어 홈 복귀 생략.")
        else:
            print("  [WRN] 로봇이 다른 작업으로 잠겨있어 홈 복귀 생략.")

        self._send_robot_operation(0, 0, 0, 0)

        try:
            print("  ➡️ OSC 서버 종료 중...")
            self.server.shutdown()
            self.server.server_close()
        except Exception as e:
            print(f"  [WRN] OSC 서버 종료 중 예외: {e}")

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
        try:
            val = float(val)
        except Exception as e:
            print("[ERR] _cb_jog: val float 변환 실패:", e); return

        if axis not in AXIS_NB:
            print(f"[WRN] 미지원 축: {axis}"); return

        try:
            if abs(val) < 1e-6:
                # --- STOP JOG ---
                if not self.is_moving_jog: return
                
                print(f"[DBG] Stop 요청 → StopJOG({JOG_REF_STOP})")
                rc = self.robot.StopJOG(JOG_REF_STOP)
                print(f"[JOG] {axis:<3} STOP")
                self.is_moving_jog = False
                if self.jog_lock.locked():
                    self.jog_lock.release()
                return

            # --- START JOG ---
            if not self.jog_lock.acquire(blocking=False):
                print(f"[JOG] 🚫 조그 시작 거부 (BUSY, 이미 다른 축 조그 중)")
                self._send_alarm(ALARM_BUSY, "BUSY (Jog Active)")
                return
            
            self.is_moving_jog = True
            nb  = AXIS_NB[axis]
            dir = 1 if val > 0 else 0
            
            rc = self.robot.StartJOG(JOG_REF_MOVE, nb, dir, JOG_MAX_DIS,
                                     self.jog_vel_pct, JOG_ACC_PCT)
            
            if rc != 0:
                print(f"[ERR] 🛑 StartJOG 실패: err={rc}")
                self.is_moving_jog = False
                if self.jog_lock.locked():
                    self.jog_lock.release()
                return

            print(f"[JOG] {axis:<3} {'+' if dir else '-'} vel={self.jog_vel_pct}%")

        except Exception as e:
            print(f"[ERR] ⚠️ _cb_jog 실행 중 예외: {e}")
            traceback.print_exc()
            self.is_moving_jog = False
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
        if self.is_moving_jog:
            print(f"[CFG] 🚫 조그 속도 변경 무시 (조그 이동 중) 요청={pct}%")
            return
        
        old = self.jog_vel_pct
        v = max(0.0, min(100.0, float(pct)))
        self.jog_vel_pct = v
        print(f"[CFG] ✅ JOG_VEL_PCT: {old:.1f}% -> {self.jog_vel_pct:.1f}%")
        
        self._emit_speed_leds()

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

    def _round_pose(self, pose_list, is_joint=False):
        """좌표값을 소수점 3자리로 반올림"""
        digits = 3
        if not pose_list or len(pose_list) < 6:
            return [0.0] * 6
        return [round(float(v), digits) for v in pose_list[:6]]

    def _load_db(self, path):
        """프리셋 JSON 파일 로드 (없으면 생성)"""
        if not os.path.exists(path):
            print(f"⚠️ [V04] DB 없음: '{path}'. 새 파일을 생성합니다.")
            try:
                if not self.robot:
                    print("🛑 [ERR] 로봇이 연결되지 않아 DB를 생성할 수 없습니다.")
                    return {}
                
                err_j, j_home = self.robot.GetActualJointPosDegree(0)
                err_c, c_home = self.robot.GetActualTCPPose(0)
                if err_j != 0 or err_c != 0:
                    print("🛑 [ERR] DB 생성 실패: 로봇 현재 위치를 읽을 수 없습니다.")
                    j_home, c_home = [0.0]*6, [0.0]*6
                
                default_data = {
                    "home": {
                        "val": self._round_pose(c_home),
                        "joint_val": self._round_pose(j_home, is_joint=True),
                        "ts": time.strftime("%Y-%m-%d %H:%M:%S")
                    }
                }
                if self._save_db(default_data):
                    print(f"✅ [V04] 새 DB 생성 완료 ('home' 슬롯에 현재 위치 저장)")
                    return default_data
                else:
                    return {}
            except Exception as e:
                print(f"🛑 [ERR] 새 DB 생성 중 예외: {e}")
                return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"⚠️ DB 로드 실패: {e}")
            return {}

    def _save_db(self, data):
        """프리셋 JSON 파일을 안전하게 저장 (임시 파일 사용)"""
        tmp = DB_FILE + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            os.replace(tmp, DB_FILE)
            return True
        except Exception as e:
            print(f"🛑 [ERR] _save_db 실패: {e}")
            return False

    # --- [V04 변경] 1단계: 현재 *조인트와 TCP* 위치 임시 저장 ---
    def _cb_record_temp(self, address, *args):
        """1단계: 현재 위치(Joint+TCP)를 'temp_pose'에 임시 저장하고 'Record-Arm' 상태로 전환"""
        
        if self.is_moving_jog or self.is_moving_preset:
            print(f"🚫 임시 저장 거부: 로봇이 이동 중입니다.")
            if self.ui_client:
                self.ui_client.send_message("/ui/record/fail", "Busy")
            return

        try:
            # 2. 로봇에서 현재 Joint와 TCP 좌표 둘 다 읽기
            err_j, j_pose = self.robot.GetActualJointPosDegree(0)
            err_c, c_pose = self.robot.GetActualTCPPose(0)
            
            if err_j != 0 or err_c != 0:
                print(f"⚠️ 좌표 읽기 실패 (J_err={err_j}, C_err={err_c})")
                if self.ui_client:
                    self.ui_client.send_message("/ui/record/fail", "GetPoseFail")
                return
            
            # 3. 상태 변수에 딕셔너리로 저장
            self.temp_pose = {
                "val": self._round_pose(c_pose),
                "joint_val": self._round_pose(j_pose, is_joint=True)
            }
            self.temp_pose_timestamp = time.time()
            
            print(f"✅ [RECORD MODE] 현재 위치 임시 저장 (Jnt/TCP 모두):")
            print(f"    JNT: {self.temp_pose['joint_val']}")
            print(f"    TCP: {self.temp_pose['val']}")
            print(f"  {RECORD_MODE_TIMEOUT}초 내에 '/robot/move [슬롯]'을 눌러 스왑하세요.")

            if self.ui_client:
                self.ui_client.send_message("/ui/record/armed", 1)

        except Exception as e:
            print(f"🛑 _cb_record_temp 처리 중 예외: {e}")
            self.temp_pose = None
            self.temp_pose_timestamp = 0.0
            
    # --- [V04 변경] 2단계: 실제 저장/스왑 로직 ---
    def _execute_save_slot(self, target_slot, slot_name, pose_dict_from_temp):
        """'저장 모드'의 실제 DB 저장 로직 (스왑 기능, Joint/TCP 모두 저장)"""
        try:
            original_pose_entry = None
            
            with self.db_lock:
                # 스왑을 위해 디스크 원본(최신)을 다시 로드
                current_db_on_disk = self._load_db(DB_FILE) 
                original_pose_entry = current_db_on_disk.get(slot_name) # 스왑을 위해 기존 값 백업

                # 새 엔트리 생성 (temp -> slot)
                new_entry_for_slot = pose_dict_from_temp.copy() # 복사본 사용
                new_entry_for_slot["ts"] = time.strftime("%Y-%m-%d %H:%M:%S")
            
                # DB 파일 및 메모리 업데이트
                current_db_on_disk[slot_name] = new_entry_for_slot # 덮어쓰기
                
                if self._save_db(current_db_on_disk):
                    self.db_data = current_db_on_disk # 메모리 DB 업데이트
                    print(f"✅ 저장 완료: '{slot_name}' -> JNT: {pose_dict_from_temp.get('joint_val')}")
                    if self.ui_client:
                        self.ui_client.send_message("/ui/record/success", target_slot)
                else:
                    print(f"🛑 저장 실패: 파일 쓰기 오류")
                    if self.ui_client:
                        self.ui_client.send_message("/ui/record/fail", "WriteError")
                    return 

            # 4. [신규] 스왑의 2단계: '기존' 좌표(val, joint_val)를 temp_pose로 이동
            if original_pose_entry:
                self.temp_pose = {
                    "val": original_pose_entry.get("val"),
                    "joint_val": original_pose_entry.get("joint_val")
                }
                self.temp_pose_timestamp = time.time() # 다시 'Armed' 상태로 만듦
                
                print(f"🔄 [SWAP] '{slot_name}'의 기존 좌표를 temp에 로드.")
                print(f"  {RECORD_MODE_TIMEOUT}초 내에 다른 슬롯에 덮어쓰세요.")
                
                if self.ui_client:
                    self.ui_client.send_message("/ui/record/armed", 1)
            else:
                print(f"ℹ️ [SWAP] '{slot_name}'에 기존 좌표가 없어 스왑(temp 로드) 생략.")

        except Exception as e:
            print(f"🛑 _execute_save_slot (swap) 처리 중 예외: {e}")
            traceback.print_exc()

    # --- [V04 변경] 기존 '이동' 로직 (MoveJ 우선) ---
    def _execute_move_slot(self, target_slot, slot_name):
        """'이동 모드'의 실제 로봇 이동 로직 (Joint 우선, Cartesian 폴백)"""
        
        target_joint_pose = None
        target_cart_pose = None
        
        with self.db_lock:
            entry = self.db_data.get(slot_name)
            if not entry:
                print(f"⚠️ '{slot_name}' 좌표 없음 ({DB_FILE})"); return
            
            target_joint_pose = entry.get("joint_val")
            target_cart_pose = entry.get("val")

        is_joint_valid = isinstance(target_joint_pose, list) and len(target_joint_pose) >= 6
        is_cart_valid = isinstance(target_cart_pose, list) and len(target_cart_pose) >= 6

        if not is_joint_valid and not is_cart_valid:
            print(f"⚠️ '{slot_name}'에 유효한 'val' 또는 'joint_val'이 없습니다."); return

        if not self.preset_lock.acquire(blocking=False):
            print("🟡 프리셋 이동 거절: BUSY (이미 다른 프리셋 이동 중)")
            self._send_alarm(ALARM_BUSY, "BUSY (Preset Move Active)")
            return

        try:
            self._emit_slot_leds(target_slot)
            self.is_moving_preset = True
            self.target_slot_ui = target_slot
            self._send_robot_operation(self.current_slot, self.target_slot_ui, 1, 0)

            move_vel = max(0.0, min(100.0, float(self.jog_vel_pct)))

            err = 0
            
            # [V04] 이동 방식 결정
            if is_joint_valid:
                print(f"🚚 프리셋 이동 시작 (MoveJ) → '{slot_name}' (슬롯 {target_slot})")
                err = self.robot.MoveJ(
                    joint_pos=target_joint_pose, 
                    tool=self.TOOL_IDX, 
                    user=self.USER_IDX, 
                    vel=move_vel
                )
            else:
                # joint_val이 없으면 val(Cartesian)로 폴백 (플립 가능성 있음)
                print(f"🚚 [경고] 프리셋 이동 (MoveCart) → '{slot_name}'. ('joint_val' 없음. 'migrate_presets_v4.py' 실행 권장)")
                err = self.robot.MoveCart(
                    desc_pos=target_cart_pose, 
                    tool=self.TOOL_IDX, 
                    user=self.USER_IDX, 
                    vel=move_vel
                )
            
            if err is None: err = 0

            if err != 0:
                print(f"🛑 이동 명령 실패 (MoveJ/MoveCart 코드 {err})")
                self.is_moving_preset = False; self.target_slot_ui = self.current_slot
                self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)
                self._send_alarm(ALARM_MOTIONFAIL, f"MOTION_FAIL(code={err})")
                self.preset_lock.release()
                return
            
            print(f"✅ 도착 확인 → '{slot_name}'")
            self.current_slot   = target_slot
            self.is_moving_preset = False
            self.target_slot_ui = self.current_slot
            self.arrived_pulse_end_time = time.time() + 1.0

        except Exception as e:
            print(f"🛑 _execute_move_slot 처리 중 예외: {e}")
            traceback.print_exc()
            self.is_moving_preset = False; self.target_slot_ui = self.current_slot
            self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)
        finally:
            if self.preset_lock.locked():
                self.preset_lock.release()

    # --- [V04 변경] 이동/저장 분기 핸들러 ---
    def _cb_move_slot(self, address, *args):
        """프리셋 슬롯 이동 OR 임시 위치 저장 (상태에 따라 분기)"""
        
        if not args: print("⚠️ /robot/move 인자 없음 (0~9 필요)"); return
        try: target_slot = int(args[0])
        except ValueError: print(f"⚠️ 잘못된 슬롯 인자: {args[0]}"); return
        if target_slot not in SLOTS: print(f"⚠️ 유효하지 않은 슬롯: {target_slot}"); return

        slot_name = SLOTS[target_slot]

        is_record_mode = False
        pose_dict_to_save = None
        
        if self.temp_pose is not None:
            pose_dict_to_save = self.temp_pose
            elapsed = time.time() - self.temp_pose_timestamp
            
            if elapsed < RECORD_MODE_TIMEOUT:
                is_record_mode = True
                print(f"ℹ️ [RECORD MODE] 감지됨 (경과: {elapsed:.1f}초). 스왑 모드로 전환.")
            else:
                print(f"ℹ️ [RECORD MODE] 시간 초과 (경과: {elapsed:.1f}초). 일반 이동 모드로 전환.")
        
        self.temp_pose = None
        self.temp_pose_timestamp = 0.0
        if self.ui_client:
            try: self.ui_client.send_message("/ui/record/armed", 0)
            except: pass

        if is_record_mode and pose_dict_to_save:
            if self.is_moving_jog or self.is_moving_preset:
                print(f"🚫 스왑 거부: 로봇이 이동 중입니다.")
                if self.ui_client: self.ui_client.send_message("/ui/record/fail", "Busy")
            else:
                self._execute_save_slot(target_slot, slot_name, pose_dict_to_save)
        else:
            self._execute_move_slot(target_slot, slot_name)

    def _emit_slot_leds(self, selected:int):
        """프리셋 슬롯 버튼 LED 갱신"""
        if not self.ui_client: return
        try:
            keys = sorted(SLOTS.keys())
            for k in keys:
                self.ui_client.send_message(f"/ui/slot/{k}", 1 if k == int(selected) else 0)
            self.ui_client.send_message("/ui/slot/value", int(selected))
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

    # OSC 종료 콜백
    def _cb_terminate(self, address, *args):
        """OSC 메시지 수신 시 서버 종료"""
        print(f"\n[STOP] 🛑 종료 명령 수신 ({address}). 서버를 종료합니다...")
        
        # self.server.shutdown()을 호출하여 serve_forever() 루프를 중지시킴
        # 그러면 serve() 함수의 finally 블록이 실행되어 _shutdown()이 호출됨.
        if self.server:
            self.server.shutdown()

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

            # --- Record-Arm 타임아웃 검사 및 해제 ---
            if self.temp_pose is not None:
                elapsed = now - self.temp_pose_timestamp
                
                # 타임아웃 시간(10.0초)을 초과했는지 확인
                if elapsed >= RECORD_MODE_TIMEOUT:
                    # 임시 데이터 초기화 (스왑 모드 해제)
                    self.temp_pose = None
                    self.temp_pose_timestamp = 0.0
                    
                    print(f"[UI] ⏳ Record-Arm 타임아웃 ({elapsed:.1f}초 경과).")

                    # UI 피드백: armed 상태를 0으로 전송하여 LED 끔
                    if self.ui_client:
                        try: 
                            self.ui_client.send_message("/ui/record/armed", 0)
                        except Exception as e:
                            print(f"[WRN] 타임아웃 해제 OSC 전송 실패: {e}")

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

                if comm_ok == 0 or estop == 1: system_state = 2
                elif collision == 1 or main_code != 0: system_state = 1
                else: system_state = 0

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

                # --- 도착 펄스 + 상태 전송 ---
                moving = 1 if (self.is_moving_preset or self.is_moving_jog) else 0

                if self.is_moving_preset:
                    target = self.target_slot_ui
                elif self.is_moving_jog:
                    target = 99
                else:
                    target = self.current_slot

                arrived_val = 0
                if time.time() < self.arrived_pulse_end_time:
                    arrived_val = 1

                self._send_robot_operation(self.current_slot, target, moving, arrived_val)

                if prev_estop is None:
                    prev_estop, prev_collision = estop, collision
                else:
                    if prev_estop != estop:
                        print("🛑 E-STOP ON" if estop else "✅ E-STOP OFF")
                        prev_estop = estop
                    if prev_collision != collision:
                        print("⚠️ COLLISION ON" if collision else "✅ COLLISION OFF")
                        prev_collision = collision

            else:
                if not self._probe_comm_ok(self.robot):
                    print("🛑 텔레메트리: 통신 오류 (Axim 전송 비활성화)")

            time.sleep(0.01)

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

# -----------------------------------------------------
# --- 엔트리포인트 ---
# -----------------------------------------------------
if __name__ == "__main__":
    srv = FR5OSCBridge()
    srv.serve()