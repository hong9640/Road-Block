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

class LaneFollowerBEVStable:
    def __init__(self):
        rospy.init_node("lane_follower_bev_stable", anonymous=True)
        rospy.loginfo("LaneFollowerBEVStable start")

        # subs / pubs
        rospy.Subscriber("/image_bev/compressed", CompressedImage, self.bev_image_callback, queue_size=1)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_callback, queue_size=1)

        self.path_pub = rospy.Publisher("/lane_path", Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)

        # --- Tunable params ---
        # BEV image -> meters conversion (픽셀->m). 환경에 맞춰 조정 필요.
        self.scale_x = 0.02   # lateral: 1 pixel -> 0.02 m (예시)
        self.scale_y = 0.03   # forward: 1 pixel -> 0.03 m (예시)
        self.standard_lane_width_m = 3.5 # Standard lane width [m]

        # Pure Pursuit / speed
        self.vehicle_length = 2.6        # 차량 길이 [m]
        self.min_lfd = 3.0           # [m]
        self.max_lfd = 18.0            # [m]
        self.lfd_gain = 0.9              # lfd = min_lfd + lfd_gain * v (m/s)
        self.target_velocity_kph = 18.0  # 목표 속도 [km/h]

        # steering limits and smoothing
        self.max_steering_rad = 0.65      # 약 37 deg
        self.steer_smoothing_alpha = 0.6 # previous*alpha + new*(1-alpha)

        # sliding window params
        self.n_windows = 10
        self.margin = 60
        self.min_pixels = 30

        # safety
        self.max_accel = 1.0
        self.max_brake = 1.0

        # state
        self.left_fit = None
        self.right_fit = None
        self.lane_path = None
        self.status_msg = None
        self.last_steering = 0.0
        self.is_lane_detected = False
        self.last_msg_time = time.time()

    def bev_image_callback(self, msg):
        self.last_msg_time = time.time()
        np_arr = np.frombuffer(msg.data, np.uint8)
        bev_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if bev_img is None:
            return

        # binary lane in BEV
        img_hsv = cv2.cvtColor(bev_img, cv2.COLOR_BGR2HSV)
        lower_wlane = np.array([0, 0, 205])
        upper_wlane = np.array([30, 60, 255])
        lower_ylane = np.array([0, 70, 120])
        upper_ylane = np.array([40, 195, 230])
        img_wlane = cv2.inRange(img_hsv, lower_wlane, upper_wlane)
        img_ylane = cv2.inRange(img_hsv, lower_ylane, upper_ylane)
        lane_binary = cv2.bitwise_or(img_wlane, img_ylane)

        # smooth binary a bit (morphology) to reduce noise
        kernel = np.ones((3,3), np.uint8)
        lane_binary = cv2.morphologyEx(lane_binary, cv2.MORPH_OPEN, kernel, iterations=1)
        lane_binary = cv2.morphologyEx(lane_binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        # sliding-window enhanced with previous-fit prediction
        h, w = lane_binary.shape
        histogram = np.sum(lane_binary[h//2:, :], axis=0)
        midpoint = histogram.shape[0] // 2
        
        left_base = np.argmax(histogram[:midpoint]) if np.max(histogram[:midpoint])>self.min_pixels else None
        right_base = (np.argmax(histogram[midpoint:]) + midpoint) if np.max(histogram[midpoint:])>self.min_pixels else None

        # Robustness check: is the distance between peaks reasonable?
        if left_base is not None and right_base is not None:
            lane_dist_pixels = right_base - left_base
            lane_dist_m = lane_dist_pixels * self.scale_x
            # Assuming typical lane width is 3.5m, with some tolerance
            if not (self.standard_lane_width_m * 0.8 <= lane_dist_m <= self.standard_lane_width_m * 1.2):
                rospy.logwarn("Detected lane width is unusual. Discarding histogram peaks for this frame.")
                left_base = None
                right_base = None

        window_height = int(h / self.n_windows)
        left_current_x = left_base if left_base is not None else (
            int(self.left_fit[0] * (h - window_height)**2 + self.left_fit[1] * (h - window_height) + self.left_fit[2]) 
            if self.left_fit is not None else None
        )
        right_current_x = right_base if right_base is not None else (
            int(self.right_fit[0] * (h - window_height)**2 + self.right_fit[1] * (h - window_height) + self.right_fit[2]) 
            if self.right_fit is not None else None
        )
        
        # If no previous fit, and no base found, try to assume a reasonable start
        if left_current_x is None and self.left_fit is None:
            left_current_x = w/2 - (self.standard_lane_width_m/2) / self.scale_x
        if right_current_x is None and self.right_fit is None:
            right_current_x = w/2 + (self.standard_lane_width_m/2) / self.scale_x

        left_xs, left_ys = [], []
        right_xs, right_ys = [], []

        for window in range(self.n_windows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height
            if win_y_low < 0: win_y_low = 0

            # predict using previous fit
            if self.left_fit is not None and left_base is None: # Use prediction only if base is not found
                pred = int(self.left_fit[0] * (win_y_high**2) + self.left_fit[1] * win_y_high + self.left_fit[2])
                left_current_x = pred
            if self.right_fit is not None and right_base is None:
                pred = int(self.right_fit[0] * (win_y_high**2) + self.right_fit[1] * win_y_high + self.right_fit[2])
                right_current_x = pred
            
            # if no base found, skip prediction zone
            if left_current_x is not None:
                lx0 = max(0, int(left_current_x - self.margin))
                lx1 = min(w, int(left_current_x + self.margin))
                if lx0 < lx1:
                    win_region = lane_binary[win_y_low:win_y_high, lx0:lx1]
                    nonzero = np.where(win_region > 0)
                    if len(nonzero[0]) > self.min_pixels:
                        mean_x = int(np.mean(nonzero[1])) + lx0
                        left_current_x = mean_x
                        left_xs.extend((nonzero[1] + lx0).tolist())
                        left_ys.extend((nonzero[0] + win_y_low).tolist())

            if right_current_x is not None:
                rx0 = max(0, int(right_current_x - self.margin))
                rx1 = min(w, int(right_current_x + self.margin))
                if rx0 < rx1:
                    win_region = lane_binary[win_y_low:win_y_high, rx0:rx1]
                    nonzero = np.where(win_region > 0)
                    if len(nonzero[0]) > self.min_pixels:
                        mean_x = int(np.mean(nonzero[1])) + rx0
                        right_current_x = mean_x
                        right_xs.extend((nonzero[1] + rx0).tolist())
                        right_ys.extend((nonzero[0] + win_y_low).tolist())

        # decide detection status
        detected_left = len(left_xs) > 50
        detected_right = len(right_xs) > 50

        center_x_pixels = None
        center_y_pixels = None

        if detected_left or detected_right:
            self.is_lane_detected = True
            try:
                if detected_left:
                    self.left_fit = np.polyfit(left_ys, left_xs, 2)
                if detected_right:
                    self.right_fit = np.polyfit(right_ys, right_xs, 2)
                
                center_y_pixels = np.linspace(h-1, h//2, 25)
                
                # Use only the detected lane or both to calculate center line
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

    def status_callback(self, msg):
        self.status_msg = msg
        self.control_loop()

    def control_loop(self):
        if rospy.is_shutdown():
            return
        
        now = time.time()
        if now - self.last_msg_time > 0.5:
            return

        if self.status_msg is None:
            return

        current_v = self.status_msg.velocity.x

        lfd = self.min_lfd + self.lfd_gain * current_v
        lfd = max(self.min_lfd, min(self.max_lfd, lfd))

        steering_cmd = 0.0

        if self.lane_path is not None and len(self.lane_path.poses) > 0:
            la_point = None
            for pose in self.lane_path.poses:
                if pose.pose.position.x >= lfd:
                    la_point = pose.pose.position
                    break
            if la_point is None:
                la_point = self.lane_path.poses[-1].pose.position

            dx = la_point.x
            dy = la_point.y

            rospy.logdebug("lookahead (m): dx=%.3f, dy=%.3f, lfd=%.3f", dx, dy, lfd)

            try:
                theta = math.atan2(dy, dx)
                steering_raw = math.atan2(2.0 * self.vehicle_length * math.sin(theta), lfd)
            except Exception as e:
                steering_raw = 0.0

            steering_raw = max(-self.max_steering_rad, min(self.max_steering_rad, steering_raw))

            steering_cmd = self.last_steering * self.steer_smoothing_alpha + steering_raw * (1.0 - self.steer_smoothing_alpha)
            self.last_steering = steering_cmd

            large_steer_thresh = 0.45
            reduce_speed_factor = 0.5
        else:
            steering_cmd = self.last_steering * 0.85
            if abs(steering_cmd) < 0.01:
                steering_cmd = 0.0
            self.last_steering = steering_cmd
            reduce_speed_factor = 0.8

        current_kph = current_v * 3.6
        vel_error = self.target_velocity_kph - current_kph
        kp = 0.06
        output = kp * vel_error
        accel = 0.0
        brake = 0.0
        if output >= 0:
            accel = min(self.max_accel, output)
            brake = 0.0
        else:
            accel = 0.0
            brake = min(self.max_brake, -output)

        if abs(steering_cmd) > 0.45:
            accel *= reduce_speed_factor

        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.steering = float(steering_cmd)
        cmd.accel = float(accel)
        cmd.brake = float(brake)
        self.ctrl_pub.publish(cmd)

        rospy.logdebug("steer=%.3f acc=%.3f brake=%.3f v=%.2f lfd=%.2f", steering_cmd, accel, brake, current_kph, lfd)

if __name__ == "__main__":
    try:
        node = LaneFollowerBEVStable()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutting down.")
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
