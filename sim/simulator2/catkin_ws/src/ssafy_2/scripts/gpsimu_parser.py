#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj

# gpsimu_parser 는 GPS, IMU 센서 데이터를 받아 차량의 상대위치를 추정하는 예제입니다.
# 노드 실행 순서
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 송신 될 Odometry 메세지 변수 생성
# 3. 위도 경도 데이터 UTM 좌표로 변환
# 4. Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
# 5. Odometry 메세지 Publish

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)

        # Subscribers & Publisher
        self.gps_sub  = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback, queue_size=10)
        self.imu_sub  = rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size=50)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # 상태 변수
        self.x, self.y = None, None
        self.lat, self.lon = 0.0, 0.0
        self.e_o, self.n_o = 0.0, 0.0
        self.is_imu = False
        self.is_gps = False

        # (1) 변환 하고자 하는 좌표계를 선언
        #  - K-City는 WGS84 / UTM Zone 52N 사용
        #  - pyproj는 (lon, lat) 순서로 넣어야 함에 유의
        #  - preserve_units=False 로 m 단위 출력
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # (2) 송신 될 Odometry 메세지 변수 생성
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"      # 부모 프레임
        self.odom_msg.child_frame_id  = "base_link" # 자식 프레임

        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            if self.is_imu and self.is_gps:
                self.convertLL2UTM()

                # (5) Odometry 메세지 Publish
                self.odom_pub.publish(self.odom_msg)

                # 간단 출력
                os.system('clear')
                print(" ROS Odometry Msgs Pose ")
                print(self.odom_msg.pose.pose.position)
                print(" ROS Odometry Msgs Orientation ")
                print(self.odom_msg.pose.pose.orientation)

            rate.sleep()

    def navsat_callback(self, gps_msg: GPSMessage):
        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset
        self.is_gps = True

    # (3) 위도 경도 데이터 UTM 좌표로 변환
    def convertLL2UTM(self):
        """
        pyproj(ProJ)를 이용하여 (lon, lat) -> (x, y) UTM 변환.
        MORAI GPS의 east/north Offset은 시뮬레이터 맵 좌표계 기준 보정값이므로 변환값에서 빼준다.
        """
        # 시뮬레이터 음영 구간 등으로 0.0이 들어올 수 있어 예외 처리
        if self.lon == 0.0 and self.lat == 0.0:
            self.x, self.y = 0.0, 0.0
        else:
            # pyproj는 (lon, lat) 순서가 정석
            utm_x, utm_y = self.proj_UTM(self.lon, self.lat)
            self.x = utm_x - self.e_o
            self.y = utm_y - self.n_o

        # (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기 (Position)
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = float(self.x)
        self.odom_msg.pose.pose.position.y = float(self.y)
        self.odom_msg.pose.pose.position.z = 0.0

    def imu_callback(self, data: Imu):
        # (4) Odometry 메세지 변수에 차량의 상태 데이터 담기 (Orientation)
        # IMU 음영 구간에서 0 quaternion 방지
        if data.orientation.w == 0.0 and \
           data.orientation.x == 0.0 and \
           data.orientation.y == 0.0 and \
           data.orientation.z == 0.0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation = data.orientation

        self.is_imu = True


if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass

