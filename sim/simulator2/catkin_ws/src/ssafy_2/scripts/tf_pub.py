#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Odometry

# 노드 실행 순서 
# 1. Callback 함수 생성
# 2. 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅

class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)

        # 브로드캐스터는 매 콜백마다 생성하지 말고 한 번만 생성
        self.br = tf.TransformBroadcaster()
        self.is_odom = False

        rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.spin()

    # (1) Callback 함수 생성
    def odom_callback(self, msg: Odometry):
        self.is_odom = True

        # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
        # Odometry 메세지 에 담긴 물체의 위치 와 자세 데이터를 아래 변수에 넣어준다.
        # 위치
        self.x = msg.pose.pose.position.x    # 물체의 x 좌표
        self.y = msg.pose.pose.position.y    # 물체의 y 좌표
        self.z = msg.pose.pose.position.z    # 필요 시 0.0으로 고정해도 됨

        # 자세 (quaternion)
        self.orientation_x = msg.pose.pose.orientation.x  # quaternion x
        self.orientation_y = msg.pose.pose.orientation.y  # quaternion y
        self.orientation_z = msg.pose.pose.orientation.z  # quaternion z
        self.orientation_w = msg.pose.pose.orientation.w  # quaternion w

        # (2) 브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅
        # TF 예제는 map 좌표를 기준으로 Ego 차량의 위치를 나타낸다.
        # 시간은 가능하면 odom 헤더의 stamp를 사용하고, 없으면 현재 시간으로 대체
        stamp = msg.header.stamp if msg.header.stamp and msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        self.br.sendTransform(
            (self.x, self.y, self.z),
            (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w),
            stamp,
            "Ego",   # child_frame_id
            "map"    # frame_id (parent)
        )

if __name__ == '__main__':
    try:
        tl = Ego_listener()
    except rospy.ROSInternalException:
        pass

