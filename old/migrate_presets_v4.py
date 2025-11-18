#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FR5 프리셋 v4 마이그레이션 스크립트 (v03 -> v04)
[cam4 전용 수정 버전]

[목적]
'cam1'에서 'cam4'로 이동한 후, 'cam4'의 자세를 확인받아 'joint_val'을 저장합니다.
'cam3' 및 다른 슬롯은 건너뜁니다.
"""

import os, sys, json, time, threading, traceback

# ---------------- Fairino SDK 경로 ----------------
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                         'fairino-python-sdk-main', 'linux', 'fairino'))
if SDK_ROOT not in sys.path:
    sys.path.insert(0, SDK_ROOT)
try:
    import Robot
    print("✅ SDK import 성공")
except Exception as e:
    print(f"🛑 SDK import 실패: {e}\n{traceback.format_exc()}")
    sys.exit(1)

# ---------------- 설정 ---------------------------
ROBOT_IP        = "192.168.2.57" # 사용자의 로봇 IP
DB_FILE         = "fr5_presets.json"
MOVE_VEL_PERCENT = 40.0 # 마이그레이션 시 이동 속도

# ---------------- 유틸리티 함수 ----------------
def _load_db(path):
    if not os.path.exists(path):
        print(f"⚠️ DB 없음: {path}")
        return None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception as e:
        print(f"⚠️ DB 로드 실패: {e}")
        return None

def _save_db(data):
    tmp = DB_FILE + ".tmp"
    try:
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        os.replace(tmp, DB_FILE)
        return True
    except Exception as e:
        print(f"🛑 [ERR] _save_db 실패: {e}")
        return False

def _round_pose(pose_list, is_joint=False):
    """좌표값을 소수점 3자리로 반올림"""
    digits = 3
    if not pose_list or len(pose_list) < 6:
        return [0.0] * 6
    return [round(float(v), digits) for v in pose_list[:6]]

# -----------------------------------------------------
# --- [수정됨] cam4 전용 마이그레이션 로직 ---
# -----------------------------------------------------
def main_migration_cam4_only():
    robot = None
    try:
        # --- 1. 로봇 연결 및 초기화 ---
        print(f"[MIG-CAM4] 🔌 FR5 연결 시도 → {ROBOT_IP}")
        robot = Robot.RPC(ROBOT_IP)
        
        print("[MIG-CAM4] 🤖 로봇 오류 리셋 및 자동 모드 활성화...")
        robot.ResetAllError()
        time.sleep(0.5)
        robot.RobotEnable(1)
        robot.Mode(0) # 자동 모드
        robot.DragTeachSwitch(0)
        time.sleep(1)
        
        tool_idx = int(getattr(robot.robot_state_pkg, "tool", 1))
        user_idx = int(getattr(robot.robot_state_pkg, "user", 0))
        print(f"[MIG-CAM4] ✅ 로봇 연결 완료 (Tool: {tool_idx}, User: {user_idx})")

        # --- 2. DB 로드 ---
        db_lock = threading.Lock()
        with db_lock:
            db_data = _load_db(DB_FILE)
            if not db_data:
                print(f"🛑 [ERR] '{DB_FILE}'을 찾을 수 없습니다. 종료합니다.")
                return

        # --- 3. 'cam1' (안전한 시작 지점) 정보 로드 ---
        slot_name_start = "cam1"
        start_pose_cart = db_data.get(slot_name_start, {}).get('val')
        if not (isinstance(start_pose_cart, list) and len(start_pose_cart) >= 6):
            print(f"🛑 [ERR] '{slot_name_start}'의 'val'을 찾을 수 없습니다. 시작할 수 없습니다.")
            return

        # --- 4. 'cam4' (목표 지점) 정보 로드 ---
        slot_name_target = "cam4"
        target_pose_cart = db_data.get(slot_name_target, {}).get('val')
        if not (isinstance(target_pose_cart, list) and len(target_pose_cart) >= 6):
            print(f"🛑 [ERR] '{slot_name_target}'의 'val'을 찾을 수 없습니다. 종료합니다.")
            return
        
        print("\n--- 🤖 [cam4] 프리셋 마이그레이션 시작 ---")
        
        # --- 5. 'cam1'으로 먼저 이동 ---
        print(f"  ➡️ 안전한 시작을 위해 '{slot_name_start}'으로 먼저 이동합니다 (MoveCart, 속도 {MOVE_VEL_PERCENT}%)")
        err_start = robot.MoveCart(
            desc_pos=start_pose_cart, 
            tool=tool_idx, 
            user=user_idx, 
            vel=MOVE_VEL_PERCENT
        )
        if err_start != 0:
            print(f"  🛑 [ERR] '{slot_name_start}'으로 이동 실패 (Code: {err_start}). 스크립트를 종료합니다.")
            return
        print("  ✅ 'cam1' 도착 완료.")
        time.sleep(1.0) # 안정화 대기

        # --- 6. 'cam4'로 이동 ---
        print(f"  ➡️ '{slot_name_target}'(으)로 이동합니다 (MoveCart, 속도 {MOVE_VEL_PERCENT}%)")
        print(f"    (i) 목표 TCP: {target_pose_cart}")
        try:
            err_target = robot.MoveCart(
                desc_pos=target_pose_cart, 
                tool=tool_idx, 
                user=user_idx, 
                vel=MOVE_VEL_PERCENT
            )
            if err_target != 0:
                print(f"  🛑 [ERR] '{slot_name_target}'(으)로 이동 실패 (Code: {err_target}).")
                return
            
            print(f"  ✅ [ {slot_name_target} ] 위치로 이동 완료.")
            time.sleep(0.5)
            
            confirm = ''
            while confirm not in ['y', 'n']:
                confirm = input("  [?] 현재 로봇의 자세가 올바릅니까? (플립이 없나요?) (y/n): ").lower()
            
            if confirm == 'y':
                err_j, j_pose = robot.GetActualJointPosDegree(0)
                if err_j != 0:
                    print("  🛑 [ERR] 현재 관절 값을 읽는 데 실패했습니다. 저장하지 않습니다.")
                    return
                    
                j_pose = _round_pose(j_pose, is_joint=True)
                with db_lock:
                    if slot_name_target not in db_data:
                        db_data[slot_name_target] = {}
                    
                    # [V04] 'joint_val'만 추가 또는 덮어쓰기
                    db_data[slot_name_target]['joint_val'] = j_pose
                    db_data[slot_name_target]['ts'] = time.strftime("%Y-%m-%d %H:%M:%S")
                    _save_db(db_data)
                print(f"  ✅ [ {slot_name_target} ] 슬롯에 관절 값 {j_pose} 를 추가했습니다.")
                print("\n--- 🤖 [cam4] 마이그레이션 완료 ---")
            
            else:
                print(f"  ➡️ 'n'을 선택했습니다. 'joint_val'을 저장하지 않습니다.")
                print(f"     스크립트를 종료합니다. 'cam3'와 'cam4'를 수동으로 수정해주세요.")

        except Exception as e:
            print(f"🛑 [ERR] MoveCart 이동 중 예외 발생: {e}")
            traceback.print_exc()

    except Exception as e:
        print(f"\n🛑 [치명적 오류] 스크립트 실행 중단: {e}")
        traceback.print_exc()
    finally:
        if robot:
            try:
                print("\n[MIG-CAM4] 🔌 로봇 연결을 해제합니다...")
                robot.RobotEnable(0)
                robot.CloseRPC()
            except Exception as e:
                print(f"  [WRN] 로봇 종료 중 오류: {e}")
        print("[MIG-CAM4] 마이그레이션 스크립트 종료.")

# -----------------------------------------------------
if __name__ == "__main__":
    main_migration_cam4_only()