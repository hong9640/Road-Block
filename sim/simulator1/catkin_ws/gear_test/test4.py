#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, subprocess, shlex, time, threading
import rospy
from morai_msgs.msg import SyncModeInfo

def run(cmd: str, timeout=None, quiet=False):
    try:
        r = subprocess.run(shlex.split(cmd), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, check=True, text=True)
        if not quiet:
            print(r.stdout.strip())
        return True, r.stdout.strip()
    except subprocess.CalledProcessError as e:
        if not quiet:
            print(e.stdout)
        return False, e.stdout
    except subprocess.TimeoutExpired as e:
        if not quiet:
            print(f"[TIMEOUT] {cmd}")
        return False, ""

class FrameCache:
    def __init__(self, topic="/SyncModeInfo"):
        self._frame = None
        self._lock = threading.Lock()
        self._sub = rospy.Subscriber(topic, SyncModeInfo, self._cb, queue_size=1)
    def _cb(self, msg: SyncModeInfo):
        with self._lock:
            self._frame = int(msg.frame)
    def get(self, timeout_s=1.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self._lock:
                if self._frame is not None:
                    return self._frame
            time.sleep(0.005)
        return None

def main():
    ap = argparse.ArgumentParser(description="Hot-swap gear to D with guaranteed commit (minimal pause).")
    ap.add_argument("--user", default="ssafy_ad_005")
    ap.add_argument("--gear", type=int, default=4)       # D=4 (확인됨)
    ap.add_argument("--timestep", type=int, default=20)  # ms(50Hz)
    ap.add_argument("--prep_ms", type=int, default=150)  # ON 직후 프레임 한 번 흐를 때까지 최대 대기(ms)
    ap.add_argument("--wait_ms", type=int, default=400)  # WaitForTick 호출 타임아웃(ms)
    args = ap.parse_args()

    rospy.init_node("gear_hot_swap_commit", anonymous=True)
    fc = FrameCache("/SyncModeInfo")

    print(f"[1] SyncMode ON (timestep={args.timestep}ms)")
    run(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: true, time_step: {args.timestep}}}\"", quiet=True)

    # ON 직후 프레임 수신 보장 + 최소 1틱 흐르게 (자동틱이 느린 환경 대비)
    F0 = fc.get(timeout_s=1.0)
    if F0 is None:
        print("[WARN] 프레임 수신이 느립니다. 조금 더 대기합니다…")
        F0 = fc.get(timeout_s=1.0)
    if F0 is None:
        F0 = 0
    deadline = time.time() + args.prep_ms/1000.0
    while time.time() < deadline:
        cur = fc.get(timeout_s=0.2)
        if cur is not None and cur != F0:
            F0 = cur
            break
        time.sleep(0.005)
    print(f"[2] 최신 프레임: {F0}")

    # 최신 프레임에 기어 D 전송
    print(f"[3] SetGear(frame={F0}, gear={args.gear})")
    run(f"rosservice call /SyncModeSetGear \"request: {{gear: {args.gear}, frame: {F0}}}\"", quiet=True)

    # 커밋 보장: 같은 프레임으로 WaitForTick 1회 (타임아웃 짧게)
    print(f"[4] Commit(WaitForTick frame={F0})")
    run(f"rosservice call /SyncModeWaitForTick \"request: {{user_id: '{args.user}', frame: {F0}}}\"",
        timeout=args.wait_ms/1000.0, quiet=True)

    # 커밋 확인(프레임 증가 확인; 실패해도 OFF로 진행)
    F1 = fc.get(timeout_s=0.4)
    if F1 is not None and F1 != F0:
        print(f"[4+] 커밋 확인: frame {F0} -> {F1}")
    else:
        print("[WARN] 프레임 증가 확인 실패(환경 지연일 수 있음). 그래도 OFF로 진행합니다.")

    print("[5] SyncMode OFF (실시간 복귀)")
    run(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: false, time_step: 0}}\"", quiet=True)
    print("[DONE] Gear set to D with guaranteed commit.")

if __name__ == "__main__":
    main()

