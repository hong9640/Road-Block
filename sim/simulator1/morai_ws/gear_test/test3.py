#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, subprocess, shlex, time

def sh(cmd: str) -> str:
    return subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT).decode("utf-8","ignore").strip()

def try_sh(cmd: str) -> bool:
    try:
        sh(cmd); return True
    except subprocess.CalledProcessError as e:
        print(f"[WARN] {cmd}\n{e.output.decode()}"); return False

def get_frame(timeout_s=0.6) -> int:
    try:
        out = subprocess.check_output(
            ["bash","-lc","rostopic echo -n 1 /SyncModeInfo 2>/dev/null | awk '/^frame:/{print $2; exit}'"],
            timeout=timeout_s
        ).decode().strip()
        return int(out) if out else -1
    except Exception:
        return -1

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--user", default="ssafy_ad_005")
    ap.add_argument("--gear", type=int, default=4)        # D=4 (당신 환경)
    ap.add_argument("--timestep", type=int, default=20)   # 자동틱 (ms) 50Hz
    ap.add_argument("--attempts", type=int, default=5)    # SetGear 재시도 횟수
    ap.add_argument("--pre_ms", type=int, default=60)     # ON 직후 안정화 대기(ms)
    ap.add_argument("--commit_ms", type=int, default=150) # 커밋 확인 최대(ms)
    args = ap.parse_args()

    print(f"[1] SyncMode ON (timestep={args.timestep}ms)")
    try_sh(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: true, time_step: {args.timestep}}}\"")

    # 아주 짧게 안정화(자동틱이 1~2번 흐르게)
    time.sleep(args.pre_ms/1000.0)

    # 최신 프레임로 SetGear 여러 번 시도 (레이스 방지)
    committed = False
    lastF = get_frame()
    for k in range(1, args.attempts+1):
        F = get_frame()
        if F < 0: 
            print("[WARN] frame 읽기 실패, 이전 값 사용"); F = lastF if lastF>=0 else 0
        print(f"[2.{k}] SetGear(frame={F}, gear={args.gear})")
        ok = try_sh(f"rosservice call /SyncModeSetGear \"request: {{gear: {args.gear}, frame: {F}}}\"")
        if not ok:
            time.sleep(0.02); lastF = F; continue

        # 자동틱으로 프레임이 넘어갈 때까지 짧게 확인
        deadline = time.time() + args.commit_ms/1000.0
        while time.time() < deadline:
            F2 = get_frame()
            if F2 >= 0 and F2 != F:
                print(f"[3] 커밋 확인: frame {F} -> {F2}")
                committed = True
                break
            time.sleep(0.01)
        if committed: break
        lastF = F

    # 최후 수단: 커밋 강제 1틱 (블로킹 최소화)
    if not committed:
        F = get_frame()
        if F < 0: F = 0
        print(f"[3*] 강제 커밋 1틱: WaitForTick(frame={F})")
        try_sh(f"rosservice call /SyncModeWaitForTick \"request: {{user_id: '{args.user}', frame: {F}}}\"")
        time.sleep(0.04)  # 한 틱 이후 반영 여유

    print("[4] SyncMode OFF (실시간 복귀)")
    try_sh(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: false, time_step: 0}}\"")
    print("[DONE] 기어 변경 시도 완료 (D가 안 보이면 attempts/commit_ms 늘려보세요)")

if __name__ == "__main__":
    main()

