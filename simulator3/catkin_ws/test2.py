#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import ros_numpy

class RoadFollower:
    def __init__(self):
        rospy.init_node('road_follower_node')

        rospy.Subscriber('/lidar3D', PointCloud2, self.lidar_callback)
        rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.obstacle_detected = False
        self.lane_offset = 0.0  # 차선에서의 차량 위치 보정값

        self.rate = rospy.Rate(10)  # 10Hz

    def lidar_callback(self, data):
        # 라이다 데이터를 numpy 배열로 변환
        pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=True)
        # 간단히 x축 전방 3m 이하의 장애물이 있으면 True 처리
        front_points = pc[(pc[:,0] > 0) & (pc[:,0] < 3.0) & (np.abs(pc[:,1]) < 1.5)]
        self.obstacle_detected = len(front_points) > 10  # 임계치 10개 점
        
    def camera_callback(self, data):
        # 압축된 이미지 데이터 디코딩
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # 차선 검출 및 중앙 편차 계산 (단순화)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5,5), 0)
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
                self.lane_offset = (lane_center - width / 2) / (width / 2)  # -1 ~ 1

    def run(self):
        while not rospy.is_shutdown():
            move_cmd = Twist()
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected ahead! Stopping.")
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
            else:
                move_cmd.linear.x = 0.4  # 전진 속도
                # 차선 중앙 보정을 위한 간단 비례제어
                kp = 0.5
                move_cmd.angular.z = -kp * self.lane_offset
            self.cmd_pub.publish(move_cmd)
            self.rate.sleep()

if __name__ == '__main__':
    follower = RoadFollower()
    follower.run()

