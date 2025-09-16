#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys
import time
import rospy
import rospkg
from math import cos, sin, pi, sqrt, pow, atan2
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import numpy as np
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class pure_pursuit:
    def __init__(self):
        rospy.init_node('pure_pursuit', anonymous=True)

        # roslaunch <arg> 로 들어오는 local path 토픽명
        arg = rospy.myargv(argv=sys.argv)
        local_path_name = arg[1]

        # (1) subscribers/publisher
        rospy.Subscriber(local_path_name, Path, self.path_callback, queue_size=1)
        rospy.Subscriber("/global_path", Path, self.global_path_callback, queue_size=1)
        rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=50)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback, queue_size=50)
        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=10)

        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1  # throttle/brake 제어 모드

        self.is_path = False
        self.is_odom = False
        self.is_status = False
        self.is_global_path = False

        self.is_look_forward_point = False

        self.forward_point = Point()
        self.current_postion = Point()

        # 차량/제어 파라미터
        self.vehicle_length = 2.6
        self.lfd = 8.0
        self.min_lfd = 5.0
        self.max_lfd = 30.0
        self.lfd_gain = 0.78  # v[m/s]에 곱해지는 gain
        self.target_velocity = 40.0  # km/h (초기값)

        self.pid = pidControl()
        self.vel_planning = velocityPlanning(self.target_velocity/3.6, 0.15)

        # 글로벌 경로가 들어올 때까지 대기 후 속도계획 1회 생성
        while not rospy.is_shutdown():
            if self.is_global_path:
                self.velocity_list = self.vel_planning.curvedBaseVelocity(self.global_path, 50)
                break
            else:
                rospy.loginfo_throttle(2.0, 'Waiting global path data')
                rospy.sleep(0.1)

        rate = rospy.Rate(30)  # 30Hz
        while not rospy.is_shutdown():
            if self.is_path and self.is_odom and self.is_status:
                self.current_waypoint = self.get_current_waypoint(self.status_msg, self.global_path)
                # 속도 계획 결과는 m/s → km/h 변환
                self.target_velocity = self.velocity_list[self.current_waypoint] * 3.6

                steering = self.calc_pure_pursuit()
                if self.is_look_forward_point:
                    self.ctrl_cmd_msg.steering = steering
                else:
                    rospy.loginfo("no found forward point")
                    self.ctrl_cmd_msg.steering = 0.0

                output = self.pid.pid(self.target_velocity, self.status_msg.velocity.x * 3.6)  # km/h 기준

                if output > 0.0:
                    self.ctrl_cmd_msg.accel = output
                    self.ctrl_cmd_msg.brake = 0.0
                else:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = -output

                # (8) 제어입력 Publish
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            rate.sleep()

    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        odom_quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_postion.x = msg.pose.pose.position.x
        self.current_postion.y = msg.pose.pose.position.y

    def status_callback(self, msg):  # Vehicl Status Subscriber
        self.is_status = True
        self.status_msg = msg

    def global_path_callback(self, msg):
        self.global_path = msg
        self.is_global_path = True

    def get_current_waypoint(self, ego_status, global_path):
        min_dist = float('inf')
        currnet_waypoint = -1
        for i, pose in enumerate(global_path.poses):
            dx = ego_status.position.x - pose.pose.position.x
            dy = ego_status.position.y - pose.pose.position.y
            dist = sqrt(dx*dx + dy*dy)
            if min_dist > dist:
                min_dist = dist
                currnet_waypoint = i
        return currnet_waypoint

    def calc_pure_pursuit(self):
        # (2) 속도 비례 LFD 설정 (속도: m/s 사용)
        v_ms = float(self.status_msg.velocity.x)
        self.lfd = self.lfd_gain * v_ms + self.min_lfd
        if self.lfd < self.min_lfd:
            self.lfd = self.min_lfd
        if self.lfd > self.max_lfd:
            self.lfd = self.max_lfd

        rospy.loginfo_throttle(1.0, f"LFD={self.lfd:.2f} (v={v_ms:.2f} m/s)")

        vehicle_position = self.current_postion
        self.is_look_forward_point = False

        # (3) 좌표 변환 행렬 (map → vehicle local)
        yaw = self.vehicle_yaw
        trans_matrix = np.array([
            [cos(yaw), -sin(yaw), vehicle_position.x],
            [sin(yaw),  cos(yaw), vehicle_position.y],
            [0.0,       0.0,      1.0]
        ])
        det_trans_matrix = np.linalg.inv(trans_matrix)

        # 경로 포인트를 차량 기준 좌표계로 변환하면서 LFD 이상 떨어진 첫 점을 forward point로 선택
        for ps in self.path.poses:
            gx = ps.pose.position.x
            gy = ps.pose.position.y
            global_path_point = np.array([gx, gy, 1.0])
            local_path_point = det_trans_matrix.dot(global_path_point)

            if local_path_point[0] > 0.0:
                dis = sqrt(local_path_point[0]**2 + local_path_point[1]**2)
                if dis >= self.lfd:
                    self.forward_point.x = local_path_point[0]
                    self.forward_point.y = local_path_point[1]
                    self.is_look_forward_point = True
                    break

        # (4) Steering 각도 계산 (Pure Pursuit)
        if self.is_look_forward_point:
            theta = atan2(self.forward_point.y, self.forward_point.x)
            # bicycle model: delta = atan2(2*L*sin(alpha)/Lfd, 1)
            steering = atan2(2.0 * self.vehicle_length * sin(theta) / self.lfd, 1.0)
        else:
            steering = 0.0

        return steering

class pidControl:
    def __init__(self):
        self.p_gain = 0.3
        self.i_gain = 0.00
        self.d_gain = 0.03
        self.prev_error = 0.0
        self.i_control = 0.0
        self.controlTime = 0.02  # 50Hz

    def pid(self, target_vel, current_vel):
        error = target_vel - current_vel  # (km/h)

        # (5) PID 제어 생성
        p_control = self.p_gain * error
        self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p_control + self.i_control + d_control
        self.prev_error = error

        # MORAI CtrlCmd의 accel/brake는 보통 [0,1] 스케일 기대 → 과도값 방지용 클램프(선택)
        output = max(min(output, 1.0), -1.0)
        return output

class velocityPlanning:
    def __init__(self, car_max_speed, road_friciton):
        self.car_max_speed = car_max_speed  # m/s
        self.road_friction = road_friciton  # μ

    def curvedBaseVelocity(self, gloabl_path, point_num):
        out_vel_plan = []

        # 시작 구간: 최대 속도
        for _ in range(0, point_num):
            out_vel_plan.append(self.car_max_speed)

        # 중간 구간: 곡률 기반 속도 계획
        for i in range(point_num, len(gloabl_path.poses) - point_num):
            x_list = []
            y_list = []
            for box in range(-point_num, point_num):
                x = gloabl_path.poses[i+box].pose.position.x
                y = gloabl_path.poses[i+box].pose.position.y
                x_list.append([-2*x, -2*y, 1])
                y_list.append(-(x*x) - (y*y))

            A = np.array(x_list, dtype=np.float64)  # [-2x, -2y, 1]
            b = np.array(y_list, dtype=np.float64)  # -(x^2 + y^2)

            # (6) 도로의 곡률 계산 (원 최소자승: x^2+y^2 + a x + b y + c = 0)
            try:
                sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)  # [a, b, c]
                a_hat, b_hat, c_hat = sol
                cx = -a_hat / 2.0
                cy = -b_hat / 2.0
                r = sqrt(max(cx*cx + cy*cy - c_hat, 1e-6))  # 안정성용 하한
            except Exception:
                r = 1e6  # 실패 시 직선에 준하는 큰 반경

            # (7) 곡률 기반 속도 계획: v_max = sqrt(r * g * μ)
            g = 9.81
            v_max = sqrt(r * g * self.road_friction)

            if v_max > self.car_max_speed:
                v_max = self.car_max_speed
            out_vel_plan.append(v_max)

        # 끝 구간: 감속 → 정지
        for _ in range(len(gloabl_path.poses) - point_num, len(gloabl_path.poses) - 10):
            out_vel_plan.append(30.0/3.6)  # 30 km/h -> m/s
        for _ in range(len(gloabl_path.poses) - 10, len(gloabl_path.poses)):
            out_vel_plan.append(0.0)

        return out_vel_plan

if __name__ == '__main__':
    try:
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass

