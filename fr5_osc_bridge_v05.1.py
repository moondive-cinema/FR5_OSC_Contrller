#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 OSC Bridge - v5.1 (Further Simplified)

[v5.1 추가 변경사항]
- MOVE_VEL_PERCENT 제거, jog_vel_pct → vel_pct로 통일
- _check_preset_format() 제거 (실행 시 불필요한 검사)
- 불필요한 빈 except 제거
- _round_pose 간소화 (is_joint 파라미터 제거)
- 변수 초기화 최적화 (사용처에서 초기화)
- 전체 코드: 899줄 → ~860줄

[기능]
1. 조그 컨트롤러: StartJOG, StopJOG
2. 프리셋 리스너: MoveJ (joint_val 우선) 또는 MoveCart (val 폴백)
3. 2단계 스왑: temp 저장 후 10초 내 슬롯 선택하면 위치 교환
"""

import os, sys, json, time, threading, traceback
from functools import partial
from contextlib import contextmanager

# ---------------- Fairino SDK 경로 ----------------
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         'fairino-python-sdk-main', 'linux', 'fairino'))
if SDK_ROOT not in sys.path:
    sys.path.insert(0, SDK_ROOT)
try:
    import Robot
except Exception as e:
    print(f"🛑 SDK import 실패: {e}")
    traceback.print_exc()
    sys.exit(1)

# ---------------- python-osc ----------------------
try:
    from pythonosc import dispatcher, osc_server, udp_client
except ImportError:
    print("🛑 python-osc 라이브러리 필요: pip install python-osc")
    sys.exit(1)

# ---------------- 설정 ---------------------------
ROBOT_IP        = "192.168.2.57"
OSC_LISTEN_IP   = "0.0.0.0"
OSC_LISTEN_PORT = 9001
AXIM_IP         = "192.168.2.50"
AXIM_PORT       = 7000
OSC_FEEDBACK_IP   = "127.0.0.1"
OSC_FEEDBACK_PORT = 9003

DB_FILE             = "fr5_presets.json"
RECORD_MODE_TIMEOUT = 10.0
SLOTS = {
    0: "home",
    1: "cam1", 2: "cam2", 3: "cam3", 4: "cam4",
    5: "cam5", 6: "cam6", 7: "cam7", 8: "cam8", 9: "cam9"
}

# 알람 코드
ALARM_OK        = 0
ALARM_ESTOP     = 1
ALARM_COLLISION = 2
ALARM_PROG_STOP = 3
ALARM_COMM_LOST = 4
ALARM_MOTIONFAIL= 5
ALARM_BUSY      = 6

# 조그 설정
JOG_REF_MOVE = 2
JOG_REF_STOP = 3
JOG_MAX_DIS  = 250.0
VEL_PCT_DEFAULT = 40.0
VEL_ACC_PCT     = 40.0
VEL_PRESETS     = (10, 20, 40, 60, 95)
AXIS_NB = {
    "x":1, "y":2, "z":3, "rx":4, "ry":5, "rz":6, "yaw":6
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

        # OSC 클라이언트
        try:
            self.axim_client = udp_client.SimpleUDPClient(AXIM_IP, AXIM_PORT)
        except Exception as e:
            self.axim_client = None
            print(f"[WRN] Aximmetry 클라이언트 생성 실패: {e}")

        try:
            self.ui_client = udp_client.SimpleUDPClient(OSC_FEEDBACK_IP, OSC_FEEDBACK_PORT)
        except Exception as e:
            self.ui_client = None
            print(f"[WRN] UI Feedback 클라이언트 생성 실패: {e}")

        # 상태 변수
        self.jog_lock = threading.Lock()
        self.is_moving_jog = False
        self.vel_pct = VEL_PCT_DEFAULT

        self.preset_lock = threading.Lock()
        self.db_lock = threading.Lock()
        self.is_moving_preset = False
        self.current_slot = 0
        self.target_slot_ui = 0
        self.arrived_pulse_end_time = 0.0
        
        with self.db_lock:
            self.db_data = self._load_db(DB_FILE)

        # 임시 저장소
        self.temp_pose = None
        self.temp_pose_timestamp = 0.0

        # 초기화
        self._init_robot()
        self._init_osc()
        self._init_telemetry_thread()

        self._emit_slot_leds(self.current_slot)
        self._emit_speed_leds()
        
        print("[DBG] 브릿지 서버 초기화 완료.")

    # -----------------------------------------------------
    # --- 유틸리티: 상태 관리 통합 ---
    # -----------------------------------------------------
    
    @property
    def is_busy(self):
        """통합 busy 체크"""
        return self.is_moving_jog or self.is_moving_preset

    @property
    def current_target(self):
        """통합 target 계산"""
        if self.is_moving_preset:
            return self.target_slot_ui
        elif self.is_moving_jog:
            return 99
        return self.current_slot

    @contextmanager
    def _acquire_lock(self, lock):
        """통일된 lock 패턴"""
        acquired = lock.acquire(blocking=False)
        if not acquired:
            raise Exception("Lock busy")
        try:
            yield
        finally:
            if lock.locked():
                lock.release()

    # -----------------------------------------------------
    # --- 1. 초기화 및 종료 ---
    # -----------------------------------------------------

    def _init_robot(self):
        """로봇 연결 및 초기화"""
        print(f"[DBG] 🔌 FR5 연결 시도 → {ROBOT_IP}")
        try:
            self.robot = Robot.RPC(ROBOT_IP)
        except Exception as e:
            print("[ERR] 🛑 Robot 연결 실패:", e)
            traceback.print_exc()
            sys.exit(1)
        
        try:
            self.robot.ResetAllError()
            time.sleep(0.5)

            self.robot.RobotEnable(1)
            self.robot.Mode(0)
            self.robot.DragTeachSwitch(0)
            self.robot.SetSpeed(30.0)

            time.sleep(1)
            print("[DBG] ✅ 로봇 자동 모드 활성화 완료")
            
            self.TOOL_IDX = int(getattr(self.robot.robot_state_pkg, "tool", 1))
            self.USER_IDX = int(getattr(self.robot.robot_state_pkg, "user", 0))
            print(f"[DBG] ℹ️ 현재 좌표계: Tool={self.TOOL_IDX}, User={self.USER_IDX}")

        except Exception as e:
            print("[ERR] ⚠️ 로봇 초기화 호출 중 예외:", e)
            traceback.print_exc()

    def _init_osc(self):
        """OSC 경로 등록"""
        for axis in AXIS_NB.keys():
            self.disp.map(f"/fr5/jog/{axis}", partial(self._cb_jog, axis=axis))

        for preset in VEL_PRESETS:
            self.disp.map(f"/fr5/jog/vel/{preset}", partial(self._cb_vel_preset, preset_val=preset))

        self.disp.map("/fr5/jog/vel", self._cb_vel_arg)
        self.disp.map("/fr5/jog/vel/get", self._get_vel)
        self.disp.map("/robot/move", self._cb_move_slot)
        self.disp.map("/robot/record/temp", self._cb_record_temp)
        self.disp.map("/robot/terminate", self._cb_terminate)
        self.disp.map("/ping", lambda a,*b: print("✅ /ping 수신"))

        self.disp.set_default_handler(lambda addr, *args: print(f"[RX] 알 수 없는 주소: {addr} {args}"))

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
        print(f"\n--- 🤖 FR5 OSC 통합 브릿지 시작 (v5.1 - Further Simplified) ---")
        print(f"  수신 대기: {self.server.server_address[0]}:{self.server.server_address[1]}")
        print(f"  로봇 IP: {ROBOT_IP} (T:{self.TOOL_IDX}, U:{self.USER_IDX})")
        print(f"  [프리셋] /robot/move [0~9], /robot/record/temp")
        print(f"  [조그] /fr5/jog/{{x,y,z,rx,ry,rz}} [1.0|0.0]")
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
        """종료 절차"""
        print("\n[DBG] _shutdown() 진입...")
        self.stop_flag = True

        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=0.5)

        print("  ➡️ UI LED 끄는 중...")
        self._emit_speed_leds_off()
        self._emit_slot_leds_off()
        if self.ui_client:
            try:
                self.ui_client.send_message("/ui/record/armed", 0)
            except:
                pass

        try:
            print("  ➡️ StopJOG 호출")
            self.robot.StopJOG(JOG_REF_STOP)
        except Exception as e:
            print(f"  [WRN] StopJOG 중 예외: {e}")

        # 홈 복귀
        home_joint_pose = None
        home_cart_pose = None
        with self.db_lock:
            home_entry = self.db_data.get("home", {})
            home_joint_pose = home_entry.get("joint_val")
            home_cart_pose = home_entry.get("val")
            
        if not self.preset_lock.locked() and not self.jog_lock.locked():
            if home_joint_pose and isinstance(home_joint_pose, list) and len(home_joint_pose) >= 6:
                print("  ➡️ 홈 위치로 복귀 (MoveJ)...")
                try:
                    self.robot.MoveJ(joint_pos=home_joint_pose, tool=self.TOOL_IDX, 
                                    user=self.USER_IDX, vel=self.vel_pct)
                    self.current_slot = 0
                except Exception as e:
                    print(f"  [WRN] 홈(MoveJ) 이동 중 예외: {e}")
            elif home_cart_pose and isinstance(home_cart_pose, list) and len(home_cart_pose) >= 6:
                print("  ➡️ 홈 위치로 복귀 (MoveCart)...")
                try:
                    self.robot.MoveCart(desc_pos=home_cart_pose, tool=self.TOOL_IDX, 
                                       user=self.USER_IDX, vel=self.vel_pct)
                    self.current_slot = 0
                except Exception as e:
                    print(f"  [WRN] 홈(MoveCart) 이동 중 예외: {e}")

        self._send_robot_operation(0, 0, 0, 0)

        try:
            print("  ➡️ OSC 서버 종료...")
            self.server.shutdown()
            self.server.server_close()
        except Exception as e:
            print(f"  [WRN] OSC 서버 종료 중 예외: {e}")

        try:
            print("  ➡️ 로봇 연결 해제")
            self.robot.RobotEnable(0)
            self.robot.CloseRPC()
        except Exception as e:
            print(f"  [WRN] CloseRPC 중 예외: {e}")
            
        print("[FR5] RPC closed. Bye.")
        sys.exit(0)

    # -----------------------------------------------------
    # --- 2. 조그 컨트롤러 ---
    # -----------------------------------------------------

    def _cb_jog(self, addr, val, axis):
        """조그 버튼 눌림/뗌 처리"""
        try:
            val = float(val)
        except Exception as e:
            print("[ERR] _cb_jog: val float 변환 실패:", e)
            return

        if axis not in AXIS_NB:
            print(f"[WRN] 미지원 축: {axis}")
            return

        try:
            if abs(val) < 1e-6:
                # STOP JOG
                if not self.is_moving_jog:
                    return
                
                self.robot.StopJOG(JOG_REF_STOP)
                print(f"[JOG] {axis:<3} STOP")
                self.is_moving_jog = False
                if self.jog_lock.locked():
                    self.jog_lock.release()
                return

            # START JOG
            with self._acquire_lock(self.jog_lock):
                self.is_moving_jog = True
                nb  = AXIS_NB[axis]
                dir = 1 if val > 0 else 0
                
                rc = self.robot.StartJOG(JOG_REF_MOVE, nb, dir, JOG_MAX_DIS,
                                         self.vel_pct, VEL_ACC_PCT)
                
                if rc != 0:
                    print(f"[ERR] 🛑 StartJOG 실패: err={rc}")
                    self.is_moving_jog = False
                    return

                print(f"[JOG] {axis:<3} {'+' if dir else '-'} vel={self.vel_pct}%")

        except Exception as e:
            if "Lock busy" in str(e):
                print(f"[JOG] 🚫 조그 시작 거부 (BUSY)")
                self._send_alarm(ALARM_BUSY, "BUSY (Jog Active)")
            else:
                print(f"[ERR] ⚠️ _cb_jog 실행 중 예외: {e}")
                traceback.print_exc()
            self.is_moving_jog = False

    def _cb_vel_preset(self, addr, *args, preset_val=None):
        """조그 속도 프리셋 버튼 처리"""
        if preset_val is not None:
            self._set_vel_pct(float(preset_val))

    def _cb_vel_arg(self, addr, *args):
        """/fr5/jog/vel <number> 형태 처리"""
        if not args:
            print("[CFG] ⚠️ /fr5/jog/vel 인자 없음")
            return
        try:
            val = float(args[0])
            self._set_vel_pct(val)
        except Exception as e:
            print(f"[CFG] ⚠️ /fr5/jog/vel 인자 변환 실패: {args} ({e})")

    def _set_vel_pct(self, pct: float):
        """속도 설정"""
        if self.is_moving_jog:
            print(f"[CFG] 🚫 속도 변경 무시 (조그 이동 중) 요청={pct}%")
            return
        
        old = self.vel_pct
        v = max(0.0, min(100.0, float(pct)))
        self.vel_pct = v
        print(f"[CFG] ✅ VEL_PCT: {old:.1f}% -> {self.vel_pct:.1f}%")
        
        self._emit_speed_leds()

    def _get_vel(self, addr, *args):
        """현재 속도 값 전송"""
        print(f"[CFG] ℹ️ VEL_PCT = {self.vel_pct:.1f}%")
        if self.ui_client:
            self.ui_client.send_message("/ui/vel/value", float(self.vel_pct))

    def _emit_speed_leds(self):
        """속도 LED 갱신"""
        if not self.ui_client:
            return
        try:
            current = float(self.vel_pct)
            for p in VEL_PRESETS:
                on = 1 if abs(current - p) <= 0.5 else 0
                self.ui_client.send_message(f"/ui/vel/{p}", on)
            self.ui_client.send_message("/ui/vel/value", current)
        except Exception as e:
            print("[WRN] UI Speed LED 갱신 실패:", e)

    def _emit_speed_leds_off(self):
        """종료 시 모든 속도 LED OFF"""
        if not self.ui_client:
            return
        try:
            for p in VEL_PRESETS:
                self.ui_client.send_message(f"/ui/vel/{p}", 0)
            self.ui_client.send_message("/ui/vel/value", 0.0)
        except Exception as e:
            print("[WRN] UI Speed LED OFF 송신 실패:", e)

    # -----------------------------------------------------
    # --- 3. 프리셋 리스너 ---
    # -----------------------------------------------------

    def _round_pose(self, pose_list):
        """좌표값을 소수점 3자리로 반올림"""
        return [round(float(v), 3) for v in pose_list[:6]]

    def _load_db(self, path):
        """프리셋 JSON 파일 로드"""
        if not os.path.exists(path):
            print(f"⚠️ [V5.1] DB 없음: '{path}'. 새 파일 생성")
            try:
                if not self.robot:
                    print("🛑 로봇 미연결로 DB 생성 불가")
                    return {}
                
                err_j, j_home = self.robot.GetActualJointPosDegree(0)
                err_c, c_home = self.robot.GetActualTCPPose(0)
                if err_j != 0 or err_c != 0:
                    j_home, c_home = [0.0]*6, [0.0]*6
                
                default_data = {
                    "home": {
                        "val": self._round_pose(c_home),
                        "joint_val": self._round_pose(j_home),
                        "ts": time.strftime("%Y-%m-%d %H:%M:%S")
                    }
                }
                if self._save_db(default_data):
                    print(f"✅ [V5.1] 새 DB 생성 완료")
                    return default_data
                else:
                    return {}
            except Exception as e:
                print(f"🛑 새 DB 생성 중 예외: {e}")
                return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as e:
            print(f"⚠️ DB 로드 실패: {e}")
            return {}

    def _save_db(self, data):
        """프리셋 JSON 파일 저장"""
        tmp = DB_FILE + ".tmp"
        try:
            with open(tmp, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            os.replace(tmp, DB_FILE)
            return True
        except Exception as e:
            print(f"🛑 _save_db 실패: {e}")
            return False

    def _cb_record_temp(self, address, *args):
        """1단계: 현재 위치를 temp에 임시 저장"""
        if self.is_busy:
            print(f"🚫 임시 저장 거부: 로봇 이동 중")
            if self.ui_client:
                self.ui_client.send_message("/ui/record/fail", "Busy")
            return

        try:
            err_j, j_pose = self.robot.GetActualJointPosDegree(0)
            err_c, c_pose = self.robot.GetActualTCPPose(0)
            
            if err_j != 0 or err_c != 0:
                print(f"⚠️ 좌표 읽기 실패 (J_err={err_j}, C_err={err_c})")
                if self.ui_client:
                    self.ui_client.send_message("/ui/record/fail", "GetPoseFail")
                return
            
            self.temp_pose = {
                "val": self._round_pose(c_pose),
                "joint_val": self._round_pose(j_pose)
            }
            self.temp_pose_timestamp = time.time()
            
            print(f"✅ [RECORD MODE] 현재 위치 임시 저장:")
            print(f"    JNT: {self.temp_pose['joint_val']}")
            print(f"    TCP: {self.temp_pose['val']}")
            print(f"  {RECORD_MODE_TIMEOUT}초 내 '/robot/move [슬롯]'로 스왑")

            if self.ui_client:
                self.ui_client.send_message("/ui/record/armed", 1)

        except Exception as e:
            print(f"🛑 _cb_record_temp 예외: {e}")
            self.temp_pose = None
            self.temp_pose_timestamp = 0.0

    def _cb_move_slot(self, address, *args):
        """프리셋 슬롯 이동 OR 스왑"""
        if not args:
            print("⚠️ /robot/move 인자 없음")
            return
        try:
            target_slot = int(args[0])
        except ValueError:
            print(f"⚠️ 잘못된 슬롯 인자: {args[0]}")
            return
        if target_slot not in SLOTS:
            print(f"⚠️ 유효하지 않은 슬롯: {target_slot}")
            return

        slot_name = SLOTS[target_slot]

        # SWAP 모드 판단
        if self._is_swap_mode():
            self._do_swap(target_slot, slot_name)
        else:
            self._execute_move_slot(target_slot, slot_name)

    def _is_swap_mode(self):
        """SWAP 모드 판단"""
        if self.temp_pose is None:
            return False
        elapsed = time.time() - self.temp_pose_timestamp
        if elapsed >= RECORD_MODE_TIMEOUT:
            # 타임아웃
            self.temp_pose = None
            self.temp_pose_timestamp = 0.0
            if self.ui_client:
                self.ui_client.send_message("/ui/record/armed", 0)
            return False
        return True

    def _do_swap(self, target_slot, slot_name):
        """SWAP 실행 - 단순화 버전"""
        if self.is_busy:
            print(f"🚫 스왑 거부: 로봇 이동 중")
            if self.ui_client:
                self.ui_client.send_message("/ui/record/fail", "Busy")
            self.temp_pose = None
            self.temp_pose_timestamp = 0.0
            if self.ui_client:
                self.ui_client.send_message("/ui/record/armed", 0)
            return

        try:
            pose_A = self.temp_pose  # 현재 temp에 저장된 위치
            
            with self.db_lock:
                # temp → slot 저장
                self.db_data[slot_name] = pose_A.copy()
                self.db_data[slot_name]["ts"] = time.strftime("%Y-%m-%d %H:%M:%S")
                
                if not self._save_db(self.db_data):
                    print(f"🛑 저장 실패: 파일 쓰기 오류")
                    if self.ui_client:
                        self.ui_client.send_message("/ui/record/fail", "WriteError")
                    return
            
            print(f"✅ 스왑 완료: temp → '{slot_name}'")
            print(f"    JNT: {pose_A.get('joint_val')}")
            
            if self.ui_client:
                self.ui_client.send_message("/ui/record/success", target_slot)
            
            # temp 클리어
            self.temp_pose = None
            self.temp_pose_timestamp = 0.0
            if self.ui_client:
                self.ui_client.send_message("/ui/record/armed", 0)

        except Exception as e:
            print(f"🛑 _do_swap 예외: {e}")
            traceback.print_exc()

    def _execute_move_slot(self, target_slot, slot_name):
        """프리셋 이동 (MoveJ 우선, MoveCart 폴백)"""
        target_joint_pose = None
        target_cart_pose = None
        
        with self.db_lock:
            entry = self.db_data.get(slot_name)
            if not entry:
                print(f"⚠️ '{slot_name}' 좌표 없음")
                return
            
            target_joint_pose = entry.get("joint_val")
            target_cart_pose = entry.get("val")

        is_joint_valid = isinstance(target_joint_pose, list) and len(target_joint_pose) >= 6
        is_cart_valid = isinstance(target_cart_pose, list) and len(target_cart_pose) >= 6

        if not is_joint_valid and not is_cart_valid:
            print(f"⚠️ '{slot_name}'에 유효한 좌표 없음")
            return

        try:
            with self._acquire_lock(self.preset_lock):
                self._emit_slot_leds(target_slot)
                self.is_moving_preset = True
                self.target_slot_ui = target_slot
                self._send_robot_operation(self.current_slot, self.target_slot_ui, 1, 0)

                move_vel = max(0.0, min(100.0, float(self.vel_pct)))

                err = 0
                
                if is_joint_valid:
                    print(f"🚚 프리셋 이동 (MoveJ) → '{slot_name}' (슬롯 {target_slot})")
                    err = self.robot.MoveJ(
                        joint_pos=target_joint_pose, 
                        tool=self.TOOL_IDX, 
                        user=self.USER_IDX, 
                        vel=move_vel
                    )
                else:
                    print(f"🚚 [경고] 프리셋 이동 (MoveCart) → '{slot_name}' (joint_val 없음)")
                    err = self.robot.MoveCart(
                        desc_pos=target_cart_pose, 
                        tool=self.TOOL_IDX, 
                        user=self.USER_IDX, 
                        vel=move_vel
                    )
                
                if err is None:
                    err = 0

                if err != 0:
                    print(f"🛑 이동 명령 실패 (코드 {err})")
                    self.is_moving_preset = False
                    self.target_slot_ui = self.current_slot
                    self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)
                    self._send_alarm(ALARM_MOTIONFAIL, f"MOTION_FAIL(code={err})")
                    return
                
                print(f"✅ 도착 확인 → '{slot_name}'")
                self.current_slot = target_slot
                self.is_moving_preset = False
                self.target_slot_ui = self.current_slot
                self.arrived_pulse_end_time = time.time() + 1.0

        except Exception as e:
            if "Lock busy" in str(e):
                print("🟡 프리셋 이동 거절: BUSY")
                self._send_alarm(ALARM_BUSY, "BUSY (Preset Move Active)")
            else:
                print(f"🛑 _execute_move_slot 예외: {e}")
                traceback.print_exc()
            self.is_moving_preset = False
            self.target_slot_ui = self.current_slot
            self._send_robot_operation(self.current_slot, self.current_slot, 0, 0)

    def _emit_slot_leds(self, selected:int):
        """프리셋 슬롯 LED 갱신"""
        if not self.ui_client:
            return
        try:
            keys = sorted(SLOTS.keys())
            for k in keys:
                self.ui_client.send_message(f"/ui/slot/{k}", 1 if k == int(selected) else 0)
            self.ui_client.send_message("/ui/slot/value", int(selected))
        except Exception as e:
            print(f"[WRN] UI Slot LED 갱신 실패: {e}")

    def _emit_slot_leds_off(self):
        """종료 시 모든 슬롯 LED OFF"""
        if not self.ui_client:
            return
        try:
            keys = sorted(SLOTS.keys())
            for k in keys:
                self.ui_client.send_message(f"/ui/slot/{k}", 0)
            self.ui_client.send_message("/ui/slot/value", -1)
        except Exception as e:
            print("[WRN] UI Slot LED OFF 송신 실패:", e)

    def _cb_terminate(self, address, *args):
        """종료 명령"""
        print(f"\n[STOP] 🛑 종료 명령 수신 ({address})")
        if self.server:
            self.server.shutdown()

    # -----------------------------------------------------
    # --- 4. 텔레메트리 (분해 버전) ---
    # -----------------------------------------------------

    def _init_telemetry_thread(self):
        """텔레메트리 스레드 시작"""
        self.telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self.telemetry_thread.start()

    def _telemetry_loop(self):
        """메인 텔레메트리 루프"""
        t_pose = t_hb = 0.0
        
        # 텔레메트리 변수 초기화
        self._last_comm_ok = None
        self._last_system_state = None
        self.seq_counter = 0
        self.last_alarm_code = None
        
        while self.robot is None and not self.stop_flag:
            time.sleep(0.1)
        
        while not self.stop_flag:
            now = time.time()

            # Record-Arm 타임아웃 체크
            self._check_record_timeout(now)

            if self.axim_client:
                # 10Hz 텔레메트리
                if now - t_pose >= 0.1:
                    self._send_telemetry()
                    t_pose = now

                # 상태 평가 및 HB
                hb_interval = self._check_and_send_status()
                if now - t_hb >= hb_interval:
                    t_hb = now

            time.sleep(0.01)

    def _check_record_timeout(self, now):
        """Record-Arm 타임아웃 체크"""
        if self.temp_pose and (now - self.temp_pose_timestamp >= RECORD_MODE_TIMEOUT):
            self.temp_pose = None
            self.temp_pose_timestamp = 0.0
            if self.ui_client:
                self.ui_client.send_message("/ui/record/armed", 0)

    def _send_telemetry(self):
        """10Hz 텔레메트리 전송"""
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
        except Exception as e:
            print(f"[WRN] 텔레메트리 전송 실패: {e}")

    def _check_and_send_status(self):
        """상태 평가, alarm, HB 전송 - 통합"""
        # 통신 상태
        comm_ok = self._probe_comm_ok(self.robot)
        if self._last_comm_ok is None:
            self._last_comm_ok = comm_ok
        elif self._last_comm_ok != comm_ok:
            print("🛑 텔레메트리: 통신 오류" if comm_ok == 0 else "✅ 텔레메트리: 통신 복구")
            self._last_comm_ok = comm_ok

        # 상태 변수
        estop = int(getattr(self.robot.robot_state_pkg, "EmergencyStop", 0)) if comm_ok else 0
        collision = int(getattr(self.robot.robot_state_pkg, "collisionState", 0)) if comm_ok else 0
        main_code = int(getattr(self.robot.robot_state_pkg, "main_code", 0)) if comm_ok else 0

        # 시스템 상태 평가
        if comm_ok == 0 or estop == 1:
            system_state = 2
        elif collision == 1 or main_code != 0:
            system_state = 1
        else:
            system_state = 0

        # HB 간격 결정
        is_any_moving = self.is_busy
        hb_interval = 5.0 if system_state in (1, 2) else (0.5 if is_any_moving else 1.0)

        # Alarm 평가
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

        if new_alarm and (new_alarm[0] != self.last_alarm_code):
            self._send_alarm(new_alarm[0], new_alarm[1])

        # 상태 변경 시 즉시 전송
        if self._last_system_state != system_state:
            self.seq_counter = (self.seq_counter + 1) & 0x7fffffff
            self._send_robot_status(system_state, self.seq_counter)
            self._last_system_state = system_state

        # robot_operation 전송
        moving = 1 if self.is_busy else 0
        target = self.current_target
        arrived_val = 1 if time.time() < self.arrived_pulse_end_time else 0
        self._send_robot_operation(self.current_slot, target, moving, arrived_val)

        return hb_interval

    def _probe_comm_ok(self, robot):
        """통신 상태 확인"""
        try:
            _ = robot.robot_state_pkg.second
            return 1
        except:
            return 0

    def _send_robot_operation(self, cur:int, tgt:int, moving:int, arrived:int):
        """robot_operation 전송"""
        if not self.axim_client:
            return
        try:
            self.axim_client.send_message("/vp/robot_operation", [int(cur), int(tgt), int(moving), int(arrived)])
        except Exception as e:
            print(f"⚠️ robot_operation 전송 실패: {e}")

    def _send_robot_status(self, system_state:int, seq:int):
        """robot_status 전송"""
        if not self.axim_client:
            return
        try:
            self.axim_client.send_message("/vp/robot_status", [int(system_state), int(seq)])
        except Exception as e:
            print(f"⚠️ robot_status 전송 실패: {e}")

    def _send_alarm(self, code:int, message:str):
        """alarm 전송"""
        if not self.axim_client:
            return
        if code == self.last_alarm_code and code != ALARM_OK:
             return
        try:
            self.axim_client.send_message("/vp/robot_alarm", [int(code), str(message)])
            self.last_alarm_code = code
        except Exception as e:
            print(f"⚠️ robot_alarm 전송 실패: {e}")

# -----------------------------------------------------
# --- 엔트리포인트 ---
# -----------------------------------------------------
if __name__ == "__main__":
    srv = FR5OSCBridge()
    srv.serve()
