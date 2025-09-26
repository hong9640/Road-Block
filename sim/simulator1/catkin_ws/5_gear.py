#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, subprocess, shlex, time

def sh(cmd: str) -> str:
    return subprocess.check_output(
        shlex.split(cmd),
        stderr=subprocess.STDOUT
    ).decode("utf-8","ignore").strip()

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
        return int(out) if out else 0
    except Exception:
        return 0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--user", default="ssafy_ad_007")
    ap.add_argument("--gear", type=int, default=4)        # D=4
    ap.add_argument("--timestep", type=int, default=20)   # 자동틱 (ms) 50Hz
    ap.add_argument("--pre_ms", type=int, default=60)     # 안정화 대기
    args = ap.parse_args()

    # Ego-0은 전역 네임스페이스(/), Ego-2는 /Ego-2/
    vehicles = [
        {"name": "Ego-0", "prefix": "/"},
        {"name": "Ego-2", "prefix": "/Ego-2/"}
    ]

    for v in vehicles:
        name = v["name"]
        prefix = v["prefix"]

        print("\n" + "="*50)
        print(f"[{name}] 기어 변경 시작")

        # SyncMode ON
        print(f"[{name}] SyncMode ON (timestep={args.timestep}ms)")
        try_sh(f"rosservice call {prefix}SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: true, time_step: {args.timestep}}}\"")

        time.sleep(args.pre_ms/1000.0)

        # 현재 프레임 얻기 (없으면 0)
        F = get_frame()
        print(f"[{name}] 현재 frame={F}")

        # SetGear 한 번만 호출
        print(f"[{name}] SetGear(frame={F}, gear={args.gear})")
        try_sh(f"rosservice call {prefix}SyncModeSetGear \"request: {{gear: {args.gear}, frame: {F}}}\"")

        # 강제로 반영
        print(f"[{name}] 강제 커밋: WaitForTick(frame={F})")
        try_sh(f"rosservice call {prefix}SyncModeWaitForTick \"request: {{user_id: '{args.user}', frame: {F}}}\"")
        time.sleep(0.04)

        # SyncMode OFF
        print(f"[{name}] SyncMode OFF (실시간 복귀)")
        try_sh(f"rosservice call {prefix}SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: false, time_step: 0}}\"")

        print(f"[{name}] DONE 기어 변경 완료")
        print("="*50)

if __name__ == "__main__":
    main()

