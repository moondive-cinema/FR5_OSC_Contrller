#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record_pose_slots_listener.py
- FR5 로봇의 현재 TCP 좌표를 실시간으로 저장하는 리스너형 유틸리티
- 명령: 0~9 (슬롯 저장), list (저장된 슬롯 보기), exit (종료)
"""

import os, sys, json, time, signal

# --- SDK 경로 (기존 구조 유지) ---
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                        'fairino-python-sdk-main', 'windows', 'fairino'))
sys.path.append(SDK_ROOT)

try:
    from Robot import RPC as FRRobot
    print("✅ SDK import 성공")
except Exception as e:
    print(f"🛑 SDK import 실패: {e}")
    sys.exit(1)

# --- 설정 ---
ROBOT_IP = "192.168.2.57"
DB_FILE  = "fr5_presets.json"
SLOTS = {
    0: "home",
    1: "cam1", 2: "cam2", 3: "cam3", 4: "cam4",
    5: "cam5", 6: "cam6", 7: "cam7", 8: "cam8", 9: "cam9"
}

# --- 유틸 함수 ---
def load_db():
    if not os.path.exists(DB_FILE): return {}
    try:
        with open(DB_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except: return {}

def save_db(data):
    tmp = DB_FILE + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    os.replace(tmp, DB_FILE)

def graceful_exit(robot):
    print("\n🔌 연결 종료 중...")
    try:
        robot.CloseRPC()
        print("✅ 세션 정상 종료")
    except Exception as e:
        print(f"⚠️ 종료 중 오류: {e}")
    sys.exit(0)

# --- 메인 ---
def main():
    # Ctrl+C 핸들링
    signal.signal(signal.SIGINT, lambda s, f: sys.exit(0))

    # 로봇 연결
    try:
        robot = FRRobot(ROBOT_IP)
        if robot is None:
            raise RuntimeError("Robot.RPC()가 None을 반환했습니다.")
        print(f"🤖 FR5 연결됨 → {ROBOT_IP}")
    except Exception as e:
        print(f"🛑 연결 실패: {e}")
        sys.exit(1)

    print("\n명령 목록:")
    print(" 0~9  → 현재 위치를 슬롯(home~cam9)에 저장")
    print(" list → 현재 저장된 슬롯 출력")
    print(" exit → 프로그램 종료\n")

    db = load_db()

    while True:
        try:
            cmd = input("명령 입력 > ").strip().lower()

            # 종료 명령
            if cmd in ["exit", "quit", "q"]:
                graceful_exit(robot)

            # 저장 슬롯 보기
            elif cmd == "list":
                db = load_db()
                if not db:
                    print("💨 저장된 좌표 없음")
                else:
                    for name, info in db.items():
                        print(f"  {name:<6} → {info['val']}")
                continue

            # 슬롯 번호
            elif cmd.isdigit() and int(cmd) in SLOTS:
                slot = SLOTS[int(cmd)]
                err, tcp = robot.GetActualTCPPose()
                if err != 0 or not tcp or len(tcp) < 6:
                    print(f"⚠️ TCP 좌표 읽기 실패 (err={err})")
                    continue
                tcp_rounded = [round(float(v), 3) for v in tcp[:6]]
                print(f"현재 좌표: {tcp_rounded}")

                # 덮어쓰기 확인
                db = load_db()
                if slot in db:
                    yn = input(f"'{slot}' 이미 존재함. 덮어쓸까요? (y/N) > ").strip().lower()
                    if yn not in ["y", "yes"]:
                        print("⏭️ 저장 취소")
                        continue

                db[slot] = {
                    "val": tcp_rounded,
                    "ts": time.strftime("%Y-%m-%d %H:%M:%S")
                }
                save_db(db)
                print(f"💾 '{slot}' 저장 완료 → {tcp_rounded}")

            else:
                print("❓ 잘못된 명령입니다. (0~9 / list / exit)")

        except KeyboardInterrupt:
            graceful_exit(robot)
        except Exception as e:
            print(f"⚠️ 예외 발생: {e}")
            time.sleep(1)

if __name__ == "__main__":
    main()
