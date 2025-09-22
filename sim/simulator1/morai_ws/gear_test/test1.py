#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gear_hot_swap.py
- SyncMode를 잠깐 ON(time_step>0)으로 켜서 시뮬레이터를 멈추지 않고
  현재 프레임에 맞춰 기어만 D(=4)로 바꾼 뒤, 자동 틱으로 커밋되면
  바로 SyncMode를 OFF(실시간 모드)로 전환합니다.
- WaitForTick를 호출하지 않아 블로킹 느낌이 거의 없습니다.
"""

import argparse
import time
import rospy
from morai_msgs.srv import SyncModeCmd, SyncModeCmdRequest, SyncModeSetGear, SyncModeSetGearRequest
from morai_msgs.msg import SyncModeInfo

def now_ms() -> int:
    return int(time.time() * 1000)

def get_frame(topic="/SyncModeInfo", timeout=0.5) -> int:
    """현재 프레임 읽기. 실패 시 -1."""
    try:
        msg = rospy.wait_for_message(topic, SyncModeInfo, timeout=timeout)
        return int(msg.frame)
    except rospy.ROSException:
        return -1

def syncmode_on(user_id: str, time_step_ms: int) -> int:
    rospy.wait_for_service("/SyncModeCmd")
    cli = rospy.ServiceProxy("/SyncModeCmd", SyncModeCmd)
    req = SyncModeCmdRequest(user_id=user_id, start_sync_mode=True, time_step=time_step_ms)
    resp = cli(req)
    if not resp.result:
        rospy.logwarn("SyncMode ON result=false (계속 진행)")
    return int(resp.frame)

def syncmode_off(user_id: str):
    rospy.wait_for_service("/SyncModeCmd")
    cli = rospy.ServiceProxy("/SyncModeCmd", SyncModeCmd)
    req = SyncModeCmdRequest(user_id=user_id, start_sync_mode=False, time_step=0)
    resp = cli(req)
    if not resp.result:
        rospy.logwarn("SyncMode OFF result=false")

def set_gear(frame: int, gear: int) -> bool:
    rospy.wait_for_service("/SyncModeSetGear")
    cli = rospy.ServiceProxy("/SyncModeSetGear", SyncModeSetGear)
    req = SyncModeSetGearRequest(gear=gear, frame=frame)
    resp = cli(req)
    if not resp.result:
        rospy.logwarn("SetGear(frame=%s, gear=%s) result=false", frame, gear)
    return bool(resp.result)

def main():
    ap = argparse.ArgumentParser(description="Hot-swap gear to D without visible pause.")
    ap.add_argument("--user", default="ssafy_ad_005", help="SyncMode user_id")
    ap.add_argument("--gear", type=int, default=4, help="기어 번호 (현재 환경에서 D=4)")
    ap.add_argument("--timestep", type=int, default=20, help="SyncMode ON 시 time_step(ms) (예: 20=50Hz)")
    ap.add_argument("--pre_ms", type=int, default=80, help="ON 직후 최신 프레임 동기화 대기(ms)")
    ap.add_argument("--commit_ms", type=int, default=100, help="SetGear 후 커밋 확인 대기(ms)")
    ap.add_argument("--keep_on", action="store_true", help="기어 바꾼 뒤 SyncMode를 유지(OFF로 전환하지 않음)")
    ap.add_argument("--info_topic", default="/SyncModeInfo", help="SyncModeInfo 토픽명")
    args = ap.parse_args()

    rospy.init_node("gear_hot_swap", anonymous=True)

    # 1) SyncMode ON (자동 틱으로 진행 유지)
    f_resp = syncmode_on(args.user, args.timestep)
    rospy.loginfo("[1] SyncMode ON (timestep=%d ms), resp.frame=%d", args.timestep, f_resp)

    # 2) 레이스 방지: 아주 짧게 최신 프레임 동기화
    f0 = get_frame(args.info_topic, timeout=0.5)
    if f0 < 0:
        rospy.logwarn("프레임을 읽지 못했습니다. 기본값 0 사용")
        f0 = 0
    deadline = now_ms() + max(0, args.pre_ms)
    F = f0
    while now_ms() < deadline:
        cur = get_frame(args.info_topic, timeout=0.15)
        if cur >= 0:
            F = cur
            # 한 번이라도 변하면 최신 프레임에 맞춤
            if cur != f0:
                break
        time.sleep(0.005)
    rospy.loginfo("[2] 최신 프레임 동기화: frame=%d", F)

    # 3) 현재 프레임에 기어 전송 (WaitForTick 호출 없음)
    ok = set_gear(F, args.gear)
    rospy.loginfo("[3] SetGear(frame=%d, gear=%d) -> %s", F, args.gear, "OK" if ok else "FAIL")

    # 4) 커밋 보장: 자동 틱으로 프레임이 한 번이라도 넘어갈 때까지 아주 짧게 확인
    committed = False
    deadline = now_ms() + max(0, args.commit_ms)
    while now_ms() < deadline:
        cur = get_frame(args.info_topic, timeout=0.15)
        if cur >= 0 and cur != F:
            committed = True
            rospy.loginfo("[4] 커밋 확인: frame %d -> %d", F, cur)
            break
        time.sleep(0.005)
    if not committed:
        rospy.logwarn("[4] 커밋 대기 타임아웃(%d ms). 지연 큰 환경이면 --commit_ms 늘리세요.", args.commit_ms)

    # 5) 마무리
    if not args.keep_on:
        syncmode_off(args.user)
        rospy.loginfo("[5] SyncMode OFF (실시간 모드 복귀). 기어=%d 유지.", args.gear)
    else:
        rospy.loginfo("[5] 요청대로 SyncMode 유지 (keep_on=True).")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

