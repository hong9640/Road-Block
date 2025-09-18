#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, subprocess, shlex, time, threading
import rospy
from morai_msgs.msg import SyncModeInfo  # 메시지 타입은 보통 제공됨

def sh(cmd: str) -> str:
    return subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT).decode("utf-8","ignore").strip()

def try_sh(cmd: str) -> bool:
    try:
        sh(cmd); return True
    except subprocess.CalledProcessError as e:
        print(f"[WARN] cmd failed: {cmd}\n{e.output.decode('utf-8','ignore')}")
        return False

class FrameCache:
    def __init__(self, topic="/SyncModeInfo"):
        self._lock = threading.Lock()
        self.frame = None
        self._sub = rospy.Subscriber(topic, SyncModeInfo, self._cb, queue_size=1)

    def _cb(self, msg: SyncModeInfo):
        with self._lock:
            self.frame = int(msg.frame)

    def get(self, timeout_s=1.0):
        """최신 프레임을 timeout 내에 얻고 반환. 없으면 None."""
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self._lock:
                if self.frame is not None:
                    return self.frame
            time.sleep(0.005)
        return None

def main():
    ap = argparse.ArgumentParser(description="Hot-swap gear to D without visible pause (stable).")
    ap.add_argument("--user", default="ssafy_ad_005")
    ap.add_argument("--gear", type=int, default=4)        # D=4 (당신 환경)
    ap.add_argument("--timestep", type=int, default=20)   # 자동 틱(ms)
    ap.add_argument("--commit_ms", type=int, default=120) # 커밋 확인 대기(ms)
    args = ap.parse_args()

    rospy.init_node("gear_hot_swap_stable", anonymous=True)
    fc = FrameCache("/SyncModeInfo")

    print(f"[1] SyncMode ON (timestep={args.timestep}ms)")
    try_sh(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: true, time_step: {args.timestep}}}\"")

    # 구독이 붙고 첫 메시지가 도착할 때까지 잠깐 대기(보통 수 ms~수십 ms)
    F = fc.get(timeout_s=1.0)
    if F is None:
        # 한 번 더 시도(환경이 느린 경우)
        time.sleep(0.1)
        F = fc.get(timeout_s=1.0)
    if F is None:
        print("[WARN] 프레임을 못 받았습니다. frame=0 사용(덜 안전)")
        F = 0
    print(f"[2] 최신 프레임: {F}")

    # 현재 보이는 최신 프레임로 기어 전송 (WaitForTick은 호출하지 않음)
    print(f"[3] SetGear(frame={F}, gear={args.gear})")
    ok = try_sh(f"rosservice call /SyncModeSetGear \"request: {{gear: {args.gear}, frame: {F}}}\"")

    # 자동틱으로 커밋됐는지 짧게 확인(프레임이 F에서 벗어났는지)
    committed = False
    deadline = time.time() + args.commit_ms/1000.0
    while time.time() < deadline:
        F2 = fc.get(timeout_s=0.2)
        if F2 is not None and F2 != F:
            print(f"[4] 커밋 확인: frame {F} -> {F2}")
            committed = True
            break
        time.sleep(0.005)

    if not committed:
        print(f"[WARN] 커밋 확인 타임아웃({args.commit_ms}ms). 환경 지연이 큰 듯합니다.")

    print("[5] SyncMode OFF (실시간 복귀)")
    try_sh(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: false, time_step: 0}}\"")
    print("[DONE] Gear hot-swap complete.")
    
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

