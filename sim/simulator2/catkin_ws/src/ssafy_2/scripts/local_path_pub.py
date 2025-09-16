#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from math import sqrt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

# local_path_pub 은 global Path (전역경로) 데이터를 받아 Local Path (지역경로) 를 만드는 예제입니다.
# Local Path (지역경로) 는 global Path(전역경로) 에서 차량과 가장 가까운 포인트를 시작으로 만들어 집니다.

class local_path_pub:
    def __init__(self):
        rospy.init_node('local_path_pub', anonymous=True)

        # (1) Global Path 와 Odometry 데이터 subscriber 생성
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=20)
        rospy.Subscriber('/global_path', Path, self.global_path_callback, queue_size=1)

        # (2) Local Path publisher 선언
        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)

        # 초기화
        self.is_odom = False
        self.is_path = False
        self.x = 0.0
        self.y = 0.0
        self.global_path_msg = Path()

        # (3) Local Path 의 Size 결정 (권장: 50~200)
        self.local_path_size = 100

        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            if self.is_odom and self.is_path and len(self.global_path_msg.poses) > 0:
                local_path_msg = Path()
                local_path_msg.header.frame_id = 'map'
                local_path_msg.header.stamp = rospy.Time.now()

                x = self.x
                y = self.y

                # (5) Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색
                min_dis = float('inf')
                current_waypoint = -1
                for idx, ps in enumerate(self.global_path_msg.poses):
                    gx = ps.pose.position.x
                    gy = ps.pose.position.y
                    dx = gx - x
                    dy = gy - y
                    d2 = dx*dx + dy*dy
                    if d2 < min_dis:
                        min_dis = d2
                        current_waypoint = idx

                # (6) 가장 가까운 포인트부터 Local Path 생성 및 예외 처리
                if current_waypoint != -1:
                    end_idx = min(current_waypoint + self.local_path_size, len(self.global_path_msg.poses))
                    # 잘라서 복사
                    for ps in self.global_path_msg.poses[current_waypoint:end_idx]:
                        cp = PoseStamped()
                        cp.header.frame_id = 'map'
                        cp.header.stamp = local_path_msg.header.stamp
                        cp.pose = ps.pose
                        local_path_msg.poses.append(cp)

                # 디버그
                # print(x, y)

                # (7) Local Path 메세지 Publish
                self.local_path_pub.publish(local_path_msg)

            rate.sleep()

    def odom_callback(self, msg: Odometry):
        self.is_odom = True
        # (4) 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장 (매 콜백 갱신)
        self.x = msg.pose.pose.position.x  # 물체의 x 좌표
        self.y = msg.pose.pose.position.y  # 물체의 y 좌표

    def global_path_callback(self, msg: Path):
        self.is_path = True
        self.global_path_msg = msg


if __name__ == '__main__':
    try:
        test_track = local_path_pub()
    except rospy.ROSInterruptException:
        pass

