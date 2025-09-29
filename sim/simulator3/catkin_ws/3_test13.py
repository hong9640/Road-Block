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
import time

class ChaseAndLaneFollower:
    def __init__(self):
        rospy.init_node("chase_and_lane_follower", anonymous=True)
        rospy.loginfo("ChaseAndLaneFollower start")

        # subs / pubs
        rospy.Subscriber("/image_bev/compressed", CompressedImage, self.bev_image_callback, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback, queue_size=1)
        rospy.Subscriber("/Ego-2/Ego_topic", EgoVehicleStatus, self.target_status_callback, queue_size=1)

        self.path_pub = rospy.Publisher("/lane_path", Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # --- 통합 제어 파라미터 ---
        self.chase_weight = 0.8   # 0.0: 100% 차선, 1.0: 100% 추격. 추격 우선을 위해 0.8로 설정
        self.target_chase_dist = 8.0 # m. 추적 시 목표 거리
        
        # --- 차선 인식/주행 파라미터 (test12.py) ---
        self.scale_x = 0.02
        self.scale_y = 0.03
        self.standard_lane_width_m = 3.5
        self.vehicle_length = 2.6
        self.min_lfd = 3.0
        self.max_lfd = 18.0
        self.lfd_gain = 0.9
        self.target_velocity_kph = 18.0
        self.max_steering_rad = 0.65
        self.steer_smoothing_alpha = 0.6
        self.n_windows = 10
        self.margin = 60
        self.min_pixels = 30
        self.max_accel = 1.0
        self.max_brake = 1.0

        # --- 차량 추적 파라미터 (2_chase.py) ---
        self.STEERING_GAIN_CHASE = 1.8
        self.VELOCITY_KP_CHASE = 0.8
        self.MAX_VELOCITY_CHASE = 11.11

        # --- state variables ---
        self.left_fit = None
        self.right_fit = None
        self.lane_path = None
        self.ego_status = None
        self.target_status = None
        self.last_steering = 0.0
        self.is_lane_detected = False
        self.last_img_time = time.time()
        self.last_control_time = time.time()
        self.is_initialized = False

    def bev_image_callback(self, msg):
        self.last_img_time = time.time()
        np_arr = np.frombuffer(msg.data, np.uint8)
        bev_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bev_img is None: return

        # 1. 색상 기반 이진화 (HSV)
        img_hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
        lower_wlane = np.array([0, 0, 205]); upper_wlane = np.array([30, 60, 255])
        lower_ylane = np.array([15, 70, 100]); upper_ylane = np.array([45, 255, 255])
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        _, img_v_binary = cv2.threshold(cv2.split(img_hsv)[2], 200, 255, cv2.THRESH_BINARY)
        img_ylane = cv2.bitwise_and(img_ylane, img_v_binary)
        lane_binary = cv2.bitwise_or(img_wlane, img_ylane)

        # 2. 모폴로지 연산
        kernel = np.ones((5,5), np.uint8)
        lane_binary = cv2.morphologyEx(lane_binary, cv2.MORPH_OPEN, kernel, iterations=1)
        lane_binary = cv2.morphologyEx(lane_binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 3. 슬라이딩 윈도우 기반 차선 탐지 및 피팅
        h, w = lane_binary.shape
        histogram = np.sum(lane_binary[h//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        left_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint]) > self.min_pixels else None
        right_base = (np.argmax(histogram[midpoint:]) + midpoint) if np.max(histogram[midpoint:]) > self.min_pixels else None
        
        window_height = int(h / self.n_windows)
        left_current_x = left_base if left_base is not None else (int(self.left_fit[0] * (h - window_height)**2 + self.left_fit[1] * (h - window_height) + self.left_fit[2]) if self.left_fit is not None else None)
        right_current_x = right_base if right_base is not None else (int(self.right_fit[0] * (h - window_height)**2 + self.right_fit[1] * (h - window_height) + self.right_fit[2]) if self.right_fit is not None else None)
        if left_current_x is None and self.left_fit is None: left_current_x = w/2 - (self.standard_lane_width_m/2) / self.scale_x
        if right_current_x is None and self.right_fit is None: right_current_x = w/2 + (self.standard_lane_width_m/2) / self.scale_x
        left_xs, left_ys, right_xs, right_ys = [], [], [], []
        for window in range(self.n_windows):
            win_y_low = h - (window + 1) * window_height; win_y_high = h - window * window_height
            if win_y_low < 0: win_y_low = 0
            if self.left_fit is not None and left_base is None: pred = int(self.left_fit[0] * (win_y_high**2) + self.left_fit[1] * win_y_high + self.left_fit[2]); left_current_x = pred
            if self.right_fit is not None and right_base is None: pred = int(self.right_fit[0] * (win_y_high**2) + self.right_fit[1] * win_y_high + self.right_fit[2]); right_current_x = pred
            if left_current_x is not None:
                lx0, lx1 = max(0, int(left_current_x - self.margin)), min(w, int(left_current_x + self.margin))
                if lx0 < lx1: win_region = lane_binary[win_y_low:win_y_high, lx0:lx1]; nonzero = np.where(win_region > 0)
                if len(nonzero[0]) > self.min_pixels: mean_x = int(np.mean(nonzero[1])) + lx0; left_current_x = mean_x; left_xs.extend((nonzero[1] + lx0).tolist()); left_ys.extend((nonzero[0] + win_y_low).tolist())
            if right_current_x is not None:
                rx0, rx1 = max(0, int(right_current_x - self.margin)), min(w, int(right_current_x + self.margin))
                if rx0 < rx1: win_region = lane_binary[win_y_low:win_y_high, rx0:rx1]; nonzero = np.where(win_region > 0)
                if len(nonzero[0]) > self.min_pixels: mean_x = int(np.mean(nonzero[1])) + rx0; right_current_x = mean_x; right_xs.extend((nonzero[1] + rx0).tolist()); right_ys.extend((nonzero[0] + win_y_low).tolist())

        detected_left = len(left_xs) > 50; detected_right = len(right_xs) > 50
        center_x_pixels = None; center_y_pixels = None
        if detected_left or detected_right:
            self.is_lane_detected = True
            try:
                if detected_left: self.left_fit = np.polyfit(left_ys, left_xs, 2)
                if detected_right: self.right_fit = np.polyfit(right_ys, right_xs, 2)
                center_y_pixels = np.linspace(h-1, h//2, 25)
                if self.left_fit is not None and self.right_fit is not None:
                    left_vals = self.left_fit[0]*center_y_pixels**2 + self.left_fit[1]*center_y_pixels + self.left_fit[2]
                    right_vals = self.right_fit[0]*center_y_pixels**2 + self.right_fit[1]*center_y_pixels + self.right_fit[2]
                    center_x_pixels = (left_vals + right_vals) / 2.0
                elif self.left_fit is not None:
                    center_x_pixels = self.left_fit[0]*center_y_pixels**2 + self.left_fit[1]*center_y_pixels + self.left_fit[2] + (self.standard_lane_width_m/2) / self.scale_x
                elif self.right_fit is not None:
                    center_x_pixels = self.right_fit[0]*center_y_pixels**2 + self.right_fit[1]*center_y_pixels + self.right_fit[2] - (self.standard_lane_width_m/2) / self.scale_x
            except np.linalg.LinAlgError:
                rospy.logwarn("Polyfit failed; skipping frame")
                self.is_lane_detected = False
                self.lane_path = None
        else:
            self.is_lane_detected = False
            self.lane_path = None
            
        if center_x_pixels is not None and center_y_pixels is not None:
            path = Path()
            path.header.frame_id = "ego"
            for px, py in zip(center_x_pixels, center_y_pixels):
                X_forward_m = (h - py) * self.scale_y
                Y_side_m = ( (w/2.0) - px ) * self.scale_x
                pose = PoseStamped()
                pose.pose.position.x = X_forward_m
                pose.pose.position.y = Y_side_m
                path.poses.append(pose)
            self.lane_path = path
            self.path_pub.publish(path)

        cv2.imshow("BEV Binary", lane_binary)
        debug_vis = cv2.cvtColor(lane_binary, cv2.COLOR_GRAY2BGR)
        if center_x_pixels is not None:
            for px, py in zip(center_x_pixels.astype(int), center_y_pixels.astype(int)):
                cv2.circle(debug_vis, (int(px), int(py)), 3, (0,255,0), -1)
        cv2.imshow("BEV Debug Center", debug_vis)
        cv2.waitKey(1)

    def ego_status_callback(self, msg):
        self.ego_status = msg
        # 이 함수에서 제어 로직을 직접 호출하여 속도 업데이트에 맞춰 제어
        self.control_loop()
        
    def target_status_callback(self, msg):
        self.target_status = msg
        
    def control_loop(self):
        now = time.time()
        # 이미지가 최신이 아니거나, 내/타겟 차량 정보가 없으면 제어하지 않음
        if now - self.last_img_time > 0.5 or self.ego_status is None or self.target_status is None:
            return

        # 1. 차량 추적 제어값 계산 (주 로직)
        dx_chase = self.target_status.position.x - self.ego_status.position.x
        dy_chase = self.target_status.position.y - self.ego_status.position.y
        distance_to_target = math.sqrt(dx_chase**2 + dy_chase**2)
        target_angle = math.atan2(dy_chase, dx_chase)
        angle_diff = target_angle - math.radians(self.ego_status.heading)
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
        
        steering_chase_cmd = self.STEERING_GAIN_CHASE * angle_diff
        steering_chase_cmd = max(min(steering_chase_cmd, 1.0), -1.0)
        
        velocity_error = distance_to_target - self.target_chase_dist
        target_velocity_chase = self.VELOCITY_KP_CHASE * velocity_error
        target_velocity_chase = max(0.0, min(target_velocity_chase, self.MAX_VELOCITY_CHASE))

        # 2. 차선 주행 제어값 계산 (보조 로직)
        steering_lane_cmd = 0.0
        if self.lane_path is not None and len(self.lane_path.poses) > 0:
            current_v = self.ego_status.velocity.x
            lfd = self.min_lfd + self.lfd_gain * current_v
            lfd = max(self.min_lfd, min(self.max_lfd, lfd))
            la_point = None
            for pose in self.lane_path.poses:
                if pose.pose.position.x >= lfd:
                    la_point = pose.pose.position
                    break
            if la_point is None: la_point = self.lane_path.poses[-1].pose.position
            dx, dy = la_point.x, la_point.y
            try:
                theta = math.atan2(dy, dx)
                steering_lane_cmd = math.atan2(2.0 * self.vehicle_length * math.sin(theta), lfd)
            except Exception:
                steering_lane_cmd = 0.0
            steering_lane_cmd = max(-self.max_steering_rad, min(self.max_steering_rad, steering_lane_cmd))

        # 3. 최종 제어 명령 가중치 적용
        final_steering = steering_chase_cmd
        if self.is_lane_detected:
            # 차선이 감지되면 추격 조향각에 차선 조향각을 보정
            weight = self.chase_weight
            final_steering = steering_chase_cmd * weight + steering_lane_cmd * (1.0 - weight)
        
        # 속도는 추격 로직을 따름
        current_v = self.ego_status.velocity.x
        vel_error_chase = target_velocity_chase - current_v
        final_accel = 0.0
        final_brake = 0.0
        if vel_error_chase >= 0:
            final_accel = min(self.max_accel, vel_error_chase * 0.5)
            final_brake = 0.0
        else:
            final_accel = 0.0
            final_brake = min(self.max_brake, -vel_error_chase * 0.5)

        # 조향값 스무딩
        final_steering = self.last_steering * self.steer_smoothing_alpha + final_steering * (1.0 - self.steer_smoothing_alpha)
        self.last_steering = final_steering
        
        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.steering = float(final_steering)
        cmd.accel = float(final_accel)
        cmd.brake = float(final_brake)
        self.ctrl_pub.publish(cmd)

        if now - self.last_control_time > 1.0:
            rospy.loginfo("Final Steer: %.3f | Accel: %.3f | Brake: %.3f | Dist: %.2f", 
                          final_steering, final_accel, final_brake, distance_to_target)
            self.last_control_time = now

if __name__ == "__main__":
    try:
        node = ChaseAndLaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutting down.")
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
