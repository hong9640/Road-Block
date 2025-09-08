#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from morai_msgs.msg import CtrlCmd
import numpy as np
import cv2
import ros_numpy
import time

class RoadFollower:
    def __init__(self):
        rospy.init_node('road_follower_node')

        rospy.Subscriber('/lidar3D', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback)

        # morai 차량 제어 토픽 (CtrlCmd)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

        self.obstacle_detected = False
        self.lane_offset = 0.0  

        # PID 제어 변수
        self.kp = 0.6
        self.ki = 0.01
        self.kd = 0.1
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

        self.rate = rospy.Rate(10)  # 10Hz

    def lidar_callback(self, data):
        pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=True)
        # 전방 1.5~5m, 좌우 ±0.3m, 높이 0.2~1.5m 영역 필터링
        front_points = pc[
            (pc[:,0] > 1.5) & (pc[:,0] < 5.0) &   
            (np.abs(pc[:,1]) < 0.3) &             
            (pc[:,2] > 0.2) & (pc[:,2] < 1.5)     
        ]
        rospy.loginfo(f"Front obstacle points count: {len(front_points)}")
        self.obstacle_detected = len(front_points) > 80  

    def camera_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 흰색 차선
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 40, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        # 노란색 차선
        lower_yellow = np.array([15, 80, 80])
        upper_yellow = np.array([35, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask_lane = cv2.bitwise_or(mask_white, mask_yellow)
        blur = cv2.GaussianBlur(mask_lane, (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)

        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height*0.6)),
            (0, int(height*0.6)),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        lines = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 50, minLineLength=40, maxLineGap=100)

        if lines is not None:
            left_lines = []
            right_lines = []
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)
                if slope < -0.3:
                    left_lines.append(line[0])
                elif slope > 0.3:
                    right_lines.append(line[0])

            def avg_line(lines):
                if len(lines) == 0:
                    return None
                x_coords = []
                y_coords = []
                for x1, y1, x2, y2 in lines:
                    x_coords.extend([x1, x2])
                    y_coords.extend([y1, y2])
                poly = np.polyfit(y_coords, x_coords, 1)
                return poly

            left_fit = avg_line(left_lines)
            right_fit = avg_line(right_lines)

            if left_fit is not None and right_fit is not None:
                y_eval = height
                left_x = left_fit[0]*y_eval + left_fit[1]
                right_x = right_fit[0]*y_eval + right_fit[1]
                lane_center = (left_x + right_x) / 2
                self.lane_offset = (lane_center - width / 2) / (width / 2)  
                rospy.loginfo(f"Lane offset: {self.lane_offset:.3f}")

    def pid_control(self, error):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.1
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        return control

    def run(self):
        while not rospy.is_shutdown():
            cmd = CtrlCmd()
            cmd.longlCmdType = 1   # 1=Throttle, 2=Velocity
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected ahead! Stopping.")
                cmd.accel = 0.0
                cmd.brake = 1.0
                cmd.steering = 0.0
            else:
                cmd.accel = 0.3   # 가속도 값 (0~1)
                cmd.brake = 0.0
                steer = self.pid_control(-self.lane_offset)
                cmd.steering = np.clip(steer, -1.0, 1.0)  
            rospy.loginfo(f"CMD -> accel:{cmd.accel:.2f}, brake:{cmd.brake:.2f}, steer:{cmd.steering:.2f}")
            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    follower = RoadFollower()
    follower.run()
