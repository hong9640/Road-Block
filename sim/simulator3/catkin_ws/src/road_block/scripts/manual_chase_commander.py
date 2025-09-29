#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import sys
import select
import tty
import termios

# 사용자에게 보여줄 안내 메시지
msg = """
-------------------------------------------------
        Manual Chase Commander v2.0
-------------------------------------------------
's' : START Chase
'x' : STOP Chase
'q' or 'ESC' to quit
-------------------------------------------------
"""

def get_key():
    """터미널에서 키 입력을 실시간으로 감지하는 함수"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def manual_commander():
    """메인 실행 함수"""
    # '/chase_status' 토픽에 String 메시지를 발행하는 Publisher 생성
    # latch=True 옵션은 가장 최근에 발행한 메시지를 계속 보관하고 있다가,
    # 새로 구독하는 노드에게 즉시 전달해주는 역할을 합니다.
    chase_pub = rospy.Publisher('/chase_status', String, queue_size=1, latch=True)
    
    # ROS 노드 초기화
    rospy.init_node('manual_chase_commander', anonymous=True)

    print(msg)
    
    try:
        while not rospy.is_shutdown():
            key = get_key()
            
            if key == 's':
                rospy.loginfo("==> Sending START command...")
                chase_pub.publish("START")
                print("==> Sent: START")

            elif key == 'x':
                rospy.loginfo("==> Sending STOP command...")
                chase_pub.publish("STOP")
                print("==> Sent: STOP")
                
            elif key == 'q' or key == '\x1b': # 'q' 또는 ESC 키
                rospy.loginfo("Exiting commander.")
                break

    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    finally:
        # 노드 종료 시 터미널 설정을 원래대로 복원
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    # 터미널의 현재 설정을 저장해 둡니다.
    settings = termios.tcgetattr(sys.stdin)
    try:
        manual_commander()
    except rospy.ROSInterruptException:
        pass
