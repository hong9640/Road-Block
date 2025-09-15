#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import math

class LaneFollowerBEV:
    def __init__(self):
        rospy.init_node("lane_follower_bev", anonymous=True)
        print("실행")

        # Subscriber
        rospy.Subscriber("/image_bev/compressed", CompressedImage, self.bev_image_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback)

        # Publisher
        self.path_pub = rospy.Publisher("/lane_path", Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # Vehicle params
        self.vehicle_length = 2.6
        self.lfd = 7.0
        self.target_velocity = 20.0
        self.min_lfd = 5.0
        self.max_lfd = 15.0
        self.lfd_gain = 0.3
        
        # 수정사항: 이전 프레임의 다항식 계수를 저장할 변수 추가
        self.left_fit = None
        self.right_fit = None

        # State variables
        self.status_msg = None
        self.lane_path = None
        self.last_steering = 0.0
        self.is_lane_detected = False

    def bev_image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        bev_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # --- HSV Lane Detection directly on the BEV image ---
        img_hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
        lower_wlane = np.array([0, 0, 205])
        upper_wlane = np.array([30, 60, 255])
        lower_ylane = np.array([0, 70, 120])
        upper_ylane = np.array([40, 195, 230])
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        lane_binary = cv2.bitwise_or(img_wlane, img_ylane)
        
        cv2.imshow("BEV Lane Binary", lane_binary)
        cv2.waitKey(1)

        # --- Enhanced Sliding Window with Prediction ---
        h, w = lane_binary.shape
        histogram = np.sum(lane_binary[h // 2:, :], axis=0)
        
        midpoint = np.int(histogram.shape[0] / 2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        n_windows = 10
        window_height = np.int(h / n_windows)
        margin = 50
        min_pixels = 50

        left_current_x = left_base
        right_current_x = right_base

        left_lane_pixels_x = []
        left_lane_pixels_y = []
        right_lane_pixels_x = []
        right_lane_pixels_y = []

        for window in range(n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            
            # 이전 프레임의 다항식으로 현재 윈도우 위치 예측
            if self.left_fit is not None:
                left_current_x = int(self.left_fit[0] * win_y_high**2 + self.left_fit[1] * win_y_high + self.left_fit[2])
            if self.right_fit is not None:
                right_current_x = int(self.right_fit[0] * win_y_high**2 + self.right_fit[1] * win_y_high + self.right_fit[2])

            left_win_x_low = left_current_x - margin
            left_win_x_high = left_current_x + margin
            
            right_win_x_low = right_current_x - margin
            right_win_x_high = right_current_x + margin

            left_win_x_low = max(0, left_win_x_low)
            left_win_x_high = min(w, left_win_x_high)
            right_win_x_low = max(0, right_win_x_low)
            right_win_x_high = min(w, right_win_x_high)
            
            pixels_left = np.where(lane_binary[win_y_low:win_y_high, left_win_x_low:left_win_x_high] > 0)
            pixels_right = np.where(lane_binary[win_y_low:win_y_high, right_win_x_low:right_win_x_high] > 0)

            if len(pixels_left[1]) > min_pixels:
                left_current_x = int(np.mean(pixels_left[1])) + left_win_x_low
                left_lane_pixels_x.extend(pixels_left[1] + left_win_x_low)
                left_lane_pixels_y.extend(pixels_left[0] + win_y_low)
            
            if len(pixels_right[1]) > min_pixels:
                right_current_x = int(np.mean(pixels_right[1])) + right_win_x_low
                right_lane_pixels_x.extend(pixels_right[1] + right_win_x_low)
                right_lane_pixels_y.extend(pixels_right[0] + win_y_low)

        # 수정사항 2: 경로 생성 조건 완화 및 중앙 경로 생성 로직 변경
        # 양쪽 또는 한쪽 차선이라도 충분히 감지되면 경로를 만듭니다.
        if len(left_lane_pixels_x) > 50 or len(right_lane_pixels_x) > 50:
            self.is_lane_detected = True
            
            try:
                # 좌우 차선에 각각 2차 다항식 피팅을 시도합니다.
                # 피팅 실패 시에는 None 값을 유지합니다.
                if len(left_lane_pixels_x) > 50:
                    self.left_fit = np.polyfit(left_lane_pixels_y, left_lane_pixels_x, 2)
                if len(right_lane_pixels_x) > 50:
                    self.right_fit = np.polyfit(right_lane_pixels_y, right_lane_pixels_x, 2)
                
                # 좌/우 피팅 결과가 모두 있어야만 중앙 경로를 생성합니다.
                # (하나라도 없으면 이전 프레임의 값을 활용하도록 수정)
                if self.left_fit is not None and self.right_fit is not None:
                    # 두 차선이 모두 감지되면 중앙점을 평균으로 계산
                    center_fit_y = np.linspace(h - 1, h // 2, 20)
                    center_x = (self.left_fit[0]*center_fit_y**2 + self.left_fit[1]*center_fit_y + self.left_fit[2] + self.right_fit[0]*center_fit_y**2 + self.right_fit[1]*center_fit_y + self.right_fit[2]) / 2
                elif self.left_fit is not None and self.right_fit is None:
                    # 왼쪽 차선만 감지되면, 왼쪽 차선과 예상되는 오른쪽 차선의 중앙점을 계산
                    center_fit_y = np.linspace(h - 1, h // 2, 20)
                    center_x = (self.left_fit[0]*center_fit_y**2 + self.left_fit[1]*center_fit_y + self.left_fit[2]) + 110 # 110은 임의의 차선 폭 절반
                elif self.right_fit is not None and self.left_fit is None:
                    # 오른쪽 차선만 감지되면, 오른쪽 차선과 예상되는 왼쪽 차선의 중앙점을 계산
                    center_fit_y = np.linspace(h - 1, h // 2, 20)
                    center_x = (self.right_fit[0]*center_fit_y**2 + self.right_fit[1]*center_fit_y + self.right_fit[2]) - 110

                path = Path()
                path.header.frame_id = "map"
                for i in range(len(center_fit_y)):
                    X_forward = float(h - center_fit_y[i]) * 0.05
                    Y_side = float(center_x[i] - w / 2) * 0.05
                    
                    pose = PoseStamped()
                    pose.pose.position.x = X_forward
                    pose.pose.position.y = Y_side
                    path.poses.append(pose)
                    
                self.lane_path = path
                self.path_pub.publish(path)

            except np.linalg.LinAlgError:
                print("다항식 피팅에 실패했습니다.")
                self.is_lane_detected = False
                self.lane_path = None
        else:
            self.lane_path = None
            self.is_lane_detected = False

    def status_callback(self, msg):
        self.status_msg = msg
        self.control_loop()

    def control_loop(self):
        if self.status_msg is None:
            return

        current_vel = self.status_msg.velocity.x * 3.6
        
        self.lfd = self.lfd_gain * current_vel + self.min_lfd
        self.lfd = min(self.lfd, self.max_lfd)

        steering = 0.0
        
        if self.is_lane_detected and self.lane_path and len(self.lane_path.poses) > 0:
            lookahead_point = self.lane_path.poses[min(len(self.lane_path.poses) - 1, 8)].pose.position
            dx = lookahead_point.x
            dy = lookahead_point.y
            
            print(f"목표 지점 (dx, dy): ({dx}, {dy})")

            theta = math.atan2(dy, dx)
            steering = math.atan2(2 * self.vehicle_length * math.sin(theta), self.lfd)
            self.last_steering = steering
        else:
            steering = self.last_steering * 0.9 if abs(self.last_steering) > 0.01 else 0.0
            self.last_steering = steering

        error = self.target_velocity - current_vel
        accel = max(0.0, 0.05 * error)
        brake = max(0.0, -0.05 * error)

        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.steering = steering
        cmd.accel = accel
        cmd.brake = brake
        
        print(f"최종 조향값: {steering}")
        
        self.ctrl_pub.publish(cmd)

if __name__ == "__main__":
    try:
        node = LaneFollowerBEV()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
