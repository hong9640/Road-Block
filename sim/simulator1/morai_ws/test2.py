#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse, subprocess, shlex, time

def sh(cmd: str) -> str:
    out = subprocess.check_output(shlex.split(cmd), stderr=subprocess.STDOUT)
    return out.decode("utf-8", errors="ignore").strip()

def try_sh(cmd: str) -> bool:
    try:
        sh(cmd); return True
    except subprocess.CalledProcessError as e:
        print(f"[WARN] cmd failed: {cmd}\n{e.output.decode()}")
        return False

def get_frame(timeout_s=0.6) -> int:
    # rostopic echo -n 1 /SyncModeInfo | awk '/^frame:/{print $2; exit}'
    try:
        out = subprocess.check_output(
            ["bash","-lc","rostopic echo -n 1 /SyncModeInfo 2>/dev/null | awk '/^frame:/{print $2; exit}'"],
            timeout=timeout_s
        ).decode().strip()
        return int(out) if out else -1
    except Exception:
        return -1

def main():
    ap = argparse.ArgumentParser(description="Hot-swap gear to D(4) without visible pause (subprocess version).")
    ap.add_argument("--user", default="ssafy_ad_005")
    ap.add_argument("--gear", type=int, default=4, help="D=4 (당신 환경 기준)")
    ap.add_argument("--timestep", type=int, default=20, help="ms; SyncMode ON 동안 자동틱")
    ap.add_argument("--pre_ms", type=int, default=80, help="ON 직후 최신 프레임 동기화 대기(ms)")
    ap.add_argument("--commit_ms", type=int, default=100, help="SetGear 후 커밋 확인 대기(ms)")
    args = ap.parse_args()

    print(f"[1] SyncMode ON (timestep={args.timestep}ms)")
    try_sh(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: true, time_step: {args.timestep}}}\"")

    # 레이스 방지: 아주 짧게 최신 프레임 잡기
    f0 = get_frame()
    if f0 < 0:
        print("[WARN] frame 읽기 실패 → 0 사용")
        f0 = 0
    F = f0
    deadline = time.time() + args.pre_ms/1000.0
    while time.time() < deadline:
        cur = get_frame()
        if cur >= 0:
            F = cur
            if cur != f0:  # 프레임 한 번 넘어갔으면 최신으로 맞춤
                break
        time.sleep(0.005)
    print(f"[2] 현재 프레임 동기화: frame={F}")

    # 현재 프레임에 기어 전송 (WaitForTick 호출 안함)
    print(f"[3] SetGear(frame={F}, gear={args.gear})")
    ok = try_sh(f"rosservice call /SyncModeSetGear \"request: {{gear: {args.gear}, frame: {F}}}\"")
    if not ok:
        print("[WARN] SetGear result=false일 수 있음(그래도 커밋 대기 진행).")

    # 커밋 보장: 자동틱으로 프레임이 1번이라도 넘어갈 때까지 아주 짧게 확인
    committed = False
    deadline = time.time() + args.commit_ms/1000.0
    while time.time() < deadline:
        cur = get_frame()
        if cur >= 0 and cur != F:
            print(f"[4] 커밋 확인: frame {F} -> {cur}")
            committed = True
            break
        time.sleep(0.005)
    if not committed:
        print(f"[WARN] 커밋 확인 타임아웃({args.commit_ms}ms). 지연 큰 환경이면 --commit_ms 늘려보세요.")

    # SyncMode OFF (실시간 복귀)
    print("[5] SyncMode OFF")
    try_sh(f"rosservice call /SyncModeCmd \"request: {{user_id: '{args.user}', start_sync_mode: false, time_step: 0}}\"")
    print(f"[DONE] Gear set to {args.gear} with minimal pause.")

if __name__ == "__main__":
    main()

