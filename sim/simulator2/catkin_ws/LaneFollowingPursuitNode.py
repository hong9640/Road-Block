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

class LaneFollowingPursuitNode:
    def __init__(self):
        rospy.init_node("lane_following_pursuit_node", anonymous=True)
        rospy.loginfo("Lane Following Pursuit Node start")

        # === Subscribers & Publishers ===
        # 이미지, 내 상태, 타겟 상태 구독
        rospy.Subscriber("/image_bev/compressed", CompressedImage, self.bev_image_callback, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback, queue_size=1)
        rospy.Subscriber("/Ego-2/Ego_topic", EgoVehicleStatus, self.target_status_callback, queue_size=1) # 도주 차량 토픽

        # 최종 제어 명령 및 경로 발행
        self.path_pub = rospy.Publisher("/lane_path", Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1) # 제어 토픽은 자신의 것에 맞게 수정

        # === 상태 변수 (State Variables) ===
        # 차선 인식 관련
        self.left_fit = None
        self.right_fit = None
        self.lane_path = None
        self.is_lane_detected = False
        
        # 내 차량 상태
        self.ego_status = None
        self.current_velocity_ms = 0.0
        
        # 타겟 차량 상태
        self.target_status = None
        
        # 제어 관련
        self.last_steering = 0.0
        self.last_msg_time = time.time()

        # === 튜닝 파라미터 (Tunable Parameters) ===
        # --- 가중치 (Weights for merging) ---
        self.W_TARGET_STEER = 0.7  # 타겟 추종 조향 가중치
        self.W_LANE_STEER = 0.3    # 차선 유지 조향 가중치

        # --- 차선 인식 (Lane Detection) ---
        self.scale_x = 0.02
        self.scale_y = 0.03
        self.standard_lane_width_m = 3.5
        self.n_windows = 10
        self.margin = 60
        self.min_pixels = 30

        # --- 차선 유지 제어 (Pure Pursuit for Lane Keeping) ---
        self.vehicle_length = 2.6
        self.min_lfd = 3.0
        self.max_lfd = 18.0
        self.lfd_gain = 0.9

        # --- 타겟 추종 제어 (Target Following) ---
        self.TARGET_DISTANCE = 10.0      # 목표 추종 거리 [m]
        self.MAX_VELOCITY_KPH = 200.0   # 최대 속도 [km/h]
        self.STEERING_GAIN = 1.2         # 타겟 추종 조향 게인
        self.VELOCITY_KP = 0.8           # 속도 제어 P-게인

        # --- 공통 제어 (Common Control) ---
        self.max_steering_rad = 0.65
        self.steer_smoothing_alpha = 0.6
        
        # 메인 제어 루프를 20Hz로 실행
        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop)

    # bev_image_callback은 원본 코드와 거의 동일하게 사용합니다.
    def bev_image_callback(self, msg):
        self.last_msg_time = time.time()
        np_arr = np.frombuffer(msg.data, np.uint8)
        bev_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bev_img is None:
            return

        # (원본 코드의 이미지 처리 로직 전체를 여기에 붙여넣기)
        # ... (생략) ...
        # 마지막 self.path_pub.publish(path) 까지 동일합니다.
        # 이 함수는 self.lane_path를 업데이트하는 역할만 충실히 수행합니다.
        img_hsv = cv.cvtColor(bev_img, cv.COLOR_BGR2HSV)
        lower_wlane = np.array([0, 0, 205])
        upper_wlane = np.array([30, 60, 255])
        lower_ylane = np.array([0, 70, 120])
        upper_ylane = np.array([40, 195, 230])
        img_wlane = cv.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv.inRange(img_hsv, lower_ylane, upper_ylane)
        lane_binary = cv.bitwise_or(img_wlane, img_ylane)
        
        kernel = np.ones((3,3), np.uint8)
        lane_binary = cv.morphologyEx(lane_binary, cv.MORPH_OPEN, kernel, iterations=1)
        lane_binary = cv.morphologyEx(lane_binary, cv.MORPH_CLOSE, kernel, iterations=1)

        h, w = lane_binary.shape
        histogram = np.sum(lane_binary[h//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        
        left_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint])>self.min_pixels else None
        right_base = (np.argmax(histogram[midpoint:]) + midpoint) if np.max(histogram[midpoint:])>self.min_pixels else None

        if left_base is not None and right_base is not None:
            lane_dist_pixels = right_base - left_base
            lane_dist_m = lane_dist_pixels * self.scale_x
            if not (self.standard_lane_width_m * 0.8 <= lane_dist_m <= self.standard_lane_width_m * 1.2):
                rospy.logwarn("Detected lane width is unusual. Discarding histogram peaks for this frame.")
                left_base = None
                right_base = None

        window_height = int(h / self.n_windows)
        left_current_x = left_base
        right_current_x = right_base
        
        if self.left_fit is not None and left_base is None:
            left_current_x = self.left_fit[0] * (h-window_height/2)**2 + self.left_fit[1]*(h-window_height/2) + self.left_fit[2]
        if self.right_fit is not None and right_base is None:
            right_current_x = self.right_fit[0] * (h-window_height/2)**2 + self.right_fit[1]*(h-window_height/2) + self.right_fit[2]

        left_xs, left_ys, right_xs, right_ys = [], [], [], []
        
        for window in range(self.n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height

            if left_current_x is not None:
                lx0, lx1 = max(0, int(left_current_x - self.margin)), min(w, int(left_current_x + self.margin))
                win_region = lane_binary[win_y_low:win_y_high, lx0:lx1]
                nonzero = np.where(win_region > 0)
                if len(nonzero[0]) > self.min_pixels:
                    mean_x = int(np.mean(nonzero[1])) + lx0
                    left_current_x = mean_x
                    left_xs.extend((nonzero[1] + lx0).tolist())
                    left_ys.extend((nonzero[0] + win_y_low).tolist())

            if right_current_x is not None:
                rx0, rx1 = max(0, int(right_current_x - self.margin)), min(w, int(right_current_x + self.margin))
                win_region = lane_binary[win_y_low:win_y_high, rx0:rx1]
                nonzero = np.where(win_region > 0)
                if len(nonzero[0]) > self.min_pixels:
                    mean_x = int(np.mean(nonzero[1])) + rx0
                    right_current_x = mean_x
                    right_xs.extend((nonzero[1] + rx0).tolist())
                    right_ys.extend((nonzero[0] + win_y_low).tolist())

        detected_left = len(left_xs) > 50
        detected_right = len(right_xs) > 50
        
        center_x_pixels, center_y_pixels = None, None

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
                self.is_lane_detected = False; self.lane_path = None
        else:
            self.is_lane_detected = False; self.lane_path = None

        if center_x_pixels is not None:
            path = Path()
            path.header.frame_id = "ego"
            for px, py in zip(center_x_pixels, center_y_pixels):
                X_forward_m = (h - py) * self.scale_y
                Y_side_m = ((w/2.0) - px) * self.scale_x
                pose = PoseStamped()
                pose.pose.position.x = X_forward_m
                pose.pose.position.y = Y_side_m
                path.poses.append(pose)
            self.lane_path = path
            self.path_pub.publish(path)

    def ego_status_callback(self, msg):
        self.ego_status = msg
        self.current_velocity_ms = msg.velocity.x

    def target_status_callback(self, msg):
        self.target_status = msg

    def control_loop(self, event):
        # 모든 정보가 수신되었는지 확인
        if self.ego_status is None or self.target_status is None:
            rospy.loginfo("Waiting for vehicle status...")
            return

        # --- 1. 타겟 추종을 위한 조향/속도 계산 ---
        steer_target = 0.0
        target_velocity_ms = 0.0
        
        dx = self.target_status.position.x - self.ego_status.position.x
        dy = self.target_status.position.y - self.ego_status.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        ego_yaw_rad = math.radians(self.ego_status.heading)
        target_angle = math.atan2(dy, dx)
        angle_diff = (target_angle - ego_yaw_rad + math.pi) % (2 * math.pi) - math.pi

        steer_target = self.STEERING_GAIN * angle_diff
        
        # 속도 제어
        dist_error = distance - self.TARGET_DISTANCE
        target_velocity_ms = self.VELOCITY_KP * dist_error
        # 최대 속도 제한
        max_vel_ms = self.MAX_VELOCITY_KPH / 3.6
        target_velocity_ms = max(0.0, min(target_velocity_ms, max_vel_ms))
        
        # --- 2. 차선 유지를 위한 조향 계산 ---
        steer_lane = 0.0
        if self.is_lane_detected and self.lane_path is not None and len(self.lane_path.poses) > 0:
            lfd = self.min_lfd + self.lfd_gain * self.current_velocity_ms
            lfd = max(self.min_lfd, min(self.max_lfd, lfd))

            la_point = None
            for pose in self.lane_path.poses:
                if pose.pose.position.x >= lfd:
                    la_point = pose.pose.position
                    break
            if la_point is None and self.lane_path.poses:
                la_point = self.lane_path.poses[-1].pose.position
            
            if la_point:
                theta = math.atan2(la_point.y, la_point.x)
                steer_lane = math.atan2(2.0 * self.vehicle_length * math.sin(theta), lfd)
        else:
            # 차선이 없으면 타겟 추종 조향만 사용하기 위해 0으로 두거나,
            # 이전 조향값을 유지하여 부드럽게 만듦
            steer_lane = self.last_steering 

        # --- 3. 조향 값 합성 및 후처리 ---
        # 가중 평균으로 최종 조향값 계산
        final_steering_raw = (self.W_TARGET_STEER * steer_target) + (self.W_LANE_STEER * steer_lane)

        # 조향값 제한
        final_steering_raw = max(-self.max_steering_rad, min(self.max_steering_rad, final_steering_raw))

        # 조향 스무딩
        final_steering_cmd = self.last_steering * self.steer_smoothing_alpha + final_steering_raw * (1.0 - self.steer_smoothing_alpha)
        self.last_steering = final_steering_cmd

        # --- 4. 최종 속도 결정 (급커브 감속 로직 추가) ---
        final_velocity_cmd = target_velocity_ms
        # 큰 조향이 필요할 경우 속도 감소
        if abs(final_steering_cmd) > 0.45: # 0.45 rad ~= 25 deg
            final_velocity_cmd *= 0.6

        # --- 5. 최종 제어 명령 발행 ---
        cmd = CtrlCmd()
        cmd.longlCmdType = 2  # 속도 제어 모드
        cmd.steering = float(final_steering_cmd)
        cmd.velocity = float(final_velocity_cmd)
        
        self.ctrl_pub.publish(cmd)
        
        rospy.loginfo(f"Steer_T: {steer_target:.2f}, Steer_L: {steer_lane:.2f} -> Final: {final_steering_cmd:.2f} | Vel: {final_velocity_cmd*3.6:.1f} km/h")


if __name__ == "__main__":
    try:
        node = LaneFollowingPursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutting down.")
    finally:
        cv2.destroyAllWindows()
