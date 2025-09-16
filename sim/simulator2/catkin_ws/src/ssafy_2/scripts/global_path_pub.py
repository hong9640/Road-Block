#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# global_path_pub 은 txt 파일로 저장한 Path 데이터를 global Path (전역경로) 로 읽어오는 예제입니다.
# 만들어진 global Path(전역경로) 는 Local Path (지역경로) 를 만드는데 사용 된다.

# 노드 실행 순서
# 1. Global Path publisher 선언 및 Global Path 변수 생성
# 2. 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
# 3. 읽어 온 경로 데이터를 Global Path 변수에 넣기
# 4. Global Path 정보 Publish


class global_path_pub:
    def __init__(self, pkg_name='ssafy_2', path_name='kcity'):
        rospy.init_node('global_path_pub', anonymous=True)

        # (1) Global Path publisher 선언 및 Global Path 변수 생성
        self.global_path_pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'map'  # 전역경로는 보통 map 프레임 기준

        # (2) 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(pkg_name)

        # 패키지 내 path 디렉토리 기준으로 파일을 찾습니다. (예: <pkg>/path/kcity.txt)
        full_path = os.path.join(pkg_path, 'path', f'{path_name}.txt')
        if not os.path.isfile(full_path):
            rospy.logerr(f'[global_path_pub] 경로 파일이 없습니다: {full_path}')
            raise FileNotFoundError(full_path)

        with open(full_path, 'r') as f:
            lines = f.readlines()

        # (3) 읽어 온 경로 데이터를 Global Path 변수에 넣기
        self.global_path_msg.poses = []
        for line in lines:
            line = line.strip()
            if not line:
                continue
            tmp = line.split()
            # 형식: x y [z], z가 없으면 0.0
            try:
                x = float(tmp[0])
                y = float(tmp[1])
                z = float(tmp[2]) if len(tmp) >= 3 else 0.0
            except (ValueError, IndexError):
                rospy.logwarn(f'[global_path_pub] 잘못된 라인 스킵: "{line}"')
                continue

            read_pose = PoseStamped()
            read_pose.header.frame_id = 'map'
            read_pose.pose.position.x = x
            read_pose.pose.position.y = y
            read_pose.pose.position.z = z
            # 방향 정보가 없으므로 단위 quaternion(0,0,0,1)
            read_pose.pose.orientation.w = 1.0

            self.global_path_msg.poses.append(read_pose)

        rospy.loginfo(f'[global_path_pub] 로드 완료: {len(self.global_path_msg.poses)} points from {full_path}')

        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # (4) Global Path 정보 Publish
            self.global_path_msg.header.stamp = rospy.Time.now()
            # 각 포즈의 timestamp도 갱신해주면 down-stream에서 시간 검사하는 노드들에 안전합니다.
            now = self.global_path_msg.header.stamp
            for ps in self.global_path_msg.poses:
                ps.header.stamp = now
            self.global_path_pub.publish(self.global_path_msg)
            rate.sleep()


if __name__ == '__main__':
    try:
        test_track = global_path_pub()
    except rospy.ROSInterruptException:
        pass

