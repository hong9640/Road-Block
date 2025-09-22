#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import math
import os
import numpy as np
from scipy.spatial import KDTree
import heapq
import cv2

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class HDMapManager:
    # ... (HDMapManager 클래스는 역할 변경이 없으므로 기존 코드와 동일합니다.)
    def __init__(self, map_dir_path):
        self.links_data = {}
        self.nodes_data = {}
        self.waypoints_xy = None
        self.kdtree = None
        self.link_info_at_waypoint = []
        self.min_x, self.min_y, self.max_x, self.max_y = float('inf'), float('inf'), float('-inf'), float('-inf')

        link_file = os.path.join(map_dir_path, 'link_set.json')
        node_file = os.path.join(map_dir_path, 'node_set.json')

        rospy.loginfo(f"맵 데이터 로딩: {link_file}, {node_file}")
        try:
            with open(link_file, 'r') as f: link_data = json.load(f)
            with open(node_file, 'r') as f: node_data = json.load(f)
        except FileNotFoundError as e:
            rospy.logerr(f"맵 파일을 찾을 수 없습니다: {e}")
            rospy.signal_shutdown("Map file not found")
            return

        for node in node_data: self.nodes_data[node['idx']] = node

        all_points_xy = []
        for link in link_data:
            link['points'] = np.array(link['points'])
            self.links_data[link['idx']] = link
            for i, p in enumerate(link['points']):
                x, y = p[0], p[1]
                self.min_x, self.min_y = min(self.min_x, x), min(self.min_y, y)
                self.max_x, self.max_y = max(self.max_y, x), max(self.max_y, y)
                all_points_xy.append([x, y])
                self.link_info_at_waypoint.append({'link_id': link['idx'], 'point_idx': i})

        if all_points_xy:
            self.waypoints_xy = np.array(all_points_xy)
            self.kdtree = KDTree(self.waypoints_xy)
            rospy.loginfo("맵 데이터 로딩 및 KDTree 생성 완료!")

    def find_projection_on_map(self, x, y):
        if self.kdtree is None: return None
        _, idx = self.kdtree.query([x, y], k=1)
        return self.link_info_at_waypoint[idx]

    def get_global_path(self, start_proj, end_proj, reverse_penalty=1.0):
        start_link_id, end_link_id = start_proj['link_id'], end_proj['link_id']

        open_set = [(0, start_link_id)]
        came_from = {}
        g_score = {link_id: float('inf') for link_id in self.links_data}; g_score[start_link_id] = 0
        f_score = {link_id: float('inf') for link_id in self.links_data}; f_score[start_link_id] = self._heuristic(start_link_id, end_link_id)

        while open_set:
            _, current_link_id = heapq.heappop(open_set)
            if current_link_id == end_link_id: return self._reconstruct_path(came_from, current_link_id)

            current_link = self.links_data[current_link_id]
            to_node_id, from_node_id = current_link['to_node_idx'], current_link['from_node_idx']

            neighbors = []
            neighbors.extend([(l_id, 'forward', current_link['link_length']) for l_id, link in self.links_data.items() if link['from_node_idx'] == to_node_id])
            neighbors.extend([(l_id, 'reverse', current_link['link_length'] * reverse_penalty) for l_id, link in self.links_data.items() if link['to_node_idx'] == from_node_id])

            for neighbor_id, direction, cost in neighbors:
                if g_score[current_link_id] + cost < g_score[neighbor_id]:
                    came_from[neighbor_id] = (current_link_id, direction)
                    g_score[neighbor_id] = g_score[current_link_id] + cost
                    f_score[neighbor_id] = g_score[neighbor_id] + self._heuristic(neighbor_id, end_link_id)
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
        return None

    def _heuristic(self, link_id_a, link_id_b):
        pos_a = self.links_data[link_id_a]['points'][-1]
        pos_b = self.links_data[link_id_b]['points'][-1]
        return np.linalg.norm(pos_a - pos_b)

    def _reconstruct_path(self, came_from, current_id):
        path_sequence = []
        while current_id in came_from:
            prev_id, direction = came_from[current_id]
            path_sequence.insert(0, (current_id, direction))
            current_id = prev_id
        path_sequence.insert(0, (current_id, 'forward'))
        return path_sequence

class PathVisualizer:
    # ... (PathVisualizer 클래스는 역할 변경이 없으므로 기존 코드와 동일합니다.)
    def __init__(self, map_manager):
        self.map_manager = map_manager
        self.SCALE = 5.0
        self.VIEW_WIDTH, self.VIEW_HEIGHT = 800, 800
        self.map_background = self._create_map_background()

    def _create_map_background(self):
        rospy.loginfo("배경 맵 이미지 생성 중...")
        map_width_m = self.map_manager.max_x - self.map_manager.min_x
        map_height_m = self.map_manager.max_y - self.map_manager.min_y
        bg_width_px = int(map_width_m * self.SCALE) + 200
        bg_height_px = int(map_height_m * self.SCALE) + 200
        background = np.zeros((bg_height_px, bg_width_px, 3), dtype=np.uint8)

        for link in self.map_manager.links_data.values():
            for i in range(len(link['points']) - 1):
                p1_x = int((link['points'][i][0] - self.map_manager.min_x) * self.SCALE) + 100
                p1_y = int((link['points'][i][1] - self.map_manager.min_y) * self.SCALE) + 100
                p2_x = int((link['points'][i+1][0] - self.map_manager.min_x) * self.SCALE) + 100
                p2_y = int((link['points'][i+1][1] - self.map_manager.min_y) * self.SCALE) + 100

                p1_y_f, p2_y_f = bg_height_px - p1_y, bg_height_px - p2_y
                cv2.line(background, (p1_x, p1_y_f), (p2_x, p2_y_f), (100, 100, 100), 1)
        rospy.loginfo("배경 맵 이미지 생성 완료!")
        return background

    def update(self, ego_status, target_status, global_path, local_path):
        ego_cx_bg = int((ego_status.position.x - self.map_manager.min_x) * self.SCALE) + 100
        ego_cy_bg_f = self.map_background.shape[0] - (int((ego_status.position.y - self.map_manager.min_y) * self.SCALE) + 100)
        try:
            view = cv2.getRectSubPix(self.map_background, (self.VIEW_WIDTH, self.VIEW_HEIGHT), (ego_cx_bg, ego_cy_bg_f))
        except cv2.error:
            view = np.zeros((self.VIEW_HEIGHT, self.VIEW_WIDTH, 3), dtype=np.uint8)

        M = cv2.getRotationMatrix2D((self.VIEW_WIDTH/2, self.VIEW_HEIGHT/2), -ego_status.heading - 90, 1)
        rotated_view = cv2.warpAffine(view, M, (self.VIEW_WIDTH, self.VIEW_HEIGHT))

        paths_to_draw = [(global_path, (0, 255, 255), 1), (local_path, (0, 255, 0), 2)]
        for path, color, thickness in paths_to_draw:
            if path:
                for i in range(len(path.poses) - 1):
                    p1 = self._world_to_rotated_pixel(path.poses[i].pose.position, ego_status)
                    p2 = self._world_to_rotated_pixel(path.poses[i+1].pose.position, ego_status)
                    cv2.line(rotated_view, p1, p2, color, thickness)

        if target_status:
            tx, ty = self._world_to_rotated_pixel(target_status.position, ego_status)
            cv2.circle(rotated_view, (tx, ty), 10, (0, 0, 255), -1)

        cv2.circle(rotated_view, (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2), 10, (255, 0, 0), -1)
        cv2.line(rotated_view, (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2), (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2 - 20), (255, 255, 0), 2)

        cv2.imshow("Hierarchical Path Planning", rotated_view)
        cv2.waitKey(1)

    def _world_to_rotated_pixel(self, pos, ego):
        dx = pos.x - ego.position.x
        dy = pos.y - ego.position.y
        yaw = math.radians(ego.heading)

        lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        pixel_x = int(self.VIEW_WIDTH/2 + ly * self.SCALE)
        pixel_y = int(self.VIEW_HEIGHT/2 - lx * self.SCALE)
        return pixel_x, pixel_y

class AggressivePursuitNode:
    """
    공격적인 추격 주행을 수행하는 메인 ROS 노드 클래스.
    """
    def __init__(self):
        rospy.init_node("aggressive_pursuit_node", anonymous=True)
        map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_data")
        self.map_manager = HDMapManager(map_dir)

        # NOTE: 이 부분이 Ego-2가 추격하는 차량이 되도록 수정되었습니다.
        # Ego-2의 상태는 추격 차량(pursuer)의 상태로, Ego의 상태는 추격 당하는 차량(target)의 상태로 설정합니다.
        self.pursuer_status, self.target_status = None, None

        # ROS Subscribers
        rospy.Subscriber("/Ego-2/Ego_topic", EgoVehicleStatus, self.pursuer_status_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.target_status_callback)

        # ROS Publishers
        self.g_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.l_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/Ego-2/ctrl_cmd', CtrlCmd, queue_size=1) # Ego-2의 제어 명령을 발행하도록 수정

        self.global_path, self.local_path = None, None
        self.last_steering = 0.0

        # --- 주행 파라미터 ---
        self.REVERSE_DRIVING_PENALTY = 1.0
        self.INTERCEPT_TIME_GAIN = 1.2
        self.LOOKAHEAD_DISTANCE = 50.0
        self.GLOBAL_PATH_UPDATE_THRESHOLD = 15.0
        self.vehicle_length = 2.6
        self.max_steering_rad = 0.85
        self.steer_smoothing_alpha = 0.3
        self.lfd_gain = 0.8
        self.min_lfd, self.max_lfd = 4.0, 30.0
        self.VELOCITY_KP = 1.5
        self.MAX_VELOCITY_KPH = 80.0

        self.AGGRESSIVE_TURN_DISTANCE = 10.0
        self.AGGRESSIVE_ANGLE_THRESHOLD = math.radians(30)
        self.MIN_AGGRESSIVE_VELOCITY_KPH = 15.0

        self.visualizer = PathVisualizer(self.map_manager)

        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop)
        rospy.on_shutdown(self.shutdown_callback)
        rospy.loginfo("Ego-2가 Ego를 추격하는 공격적 최단경로 추격 노드 시작.")

    def shutdown_callback(self):
        rospy.loginfo("노드 종료. OpenCV 창 닫기.")
        cv2.destroyAllWindows()

    def pursuer_status_callback(self, msg): self.pursuer_status = msg
    def target_status_callback(self, msg): self.target_status = msg

    def control_loop(self, event):
        if not self.pursuer_status or not self.target_status: return

        ego_proj = self.map_manager.find_projection_on_map(self.pursuer_status.position.x, self.pursuer_status.position.y)
        target_proj = self.map_manager.find_projection_on_map(self.target_status.position.x, self.target_status.position.y)
        if not ego_proj or not target_proj: return

        goal_proj = self._get_intercept_goal_projection(target_proj)

        if self.global_path is None or self._should_update_global_path(goal_proj):
            rospy.loginfo("전역 경로를 재탐색합니다...")
            path_seq = self.map_manager.get_global_path(ego_proj, goal_proj, self.REVERSE_DRIVING_PENALTY)
            if path_seq:
                self.global_path = self._stitch_links_to_path(path_seq, ego_proj)
                self.g_path_pub.publish(self.global_path)
            else: self.global_path = None

        if not self.global_path: return

        self.local_path = self._extract_local_path()
        if not self.local_path: return
        self.l_path_pub.publish(self.local_path)

        steering_cmd = self.calculate_steering()
        velocity_cmd = self.calculate_velocity(goal_proj)

        cmd = CtrlCmd(longlCmdType=2, steering=float(steering_cmd), velocity=float(velocity_cmd))
        self.ctrl_pub.publish(cmd)

        self.visualizer.update(self.pursuer_status, self.target_status, self.global_path, self.local_path)

    def _get_intercept_goal_projection(self, target_proj):
        dx = self.target_status.position.x - self.pursuer_status.position.x
        dy = self.target_status.position.y - self.pursuer_status.position.y
        dist_direct = math.sqrt(dx**2 + dy**2)

        ego_speed = self.pursuer_status.velocity.x
        if ego_speed < 1.0:
            ego_speed = 1.0

        time_to_intercept = (dist_direct / ego_speed) * self.INTERCEPT_TIME_GAIN

        target_speed = self.target_status.velocity.x
        intercept_distance_ahead = target_speed * time_to_intercept

        link_id = target_proj['link_id']
        start_idx = target_proj['point_idx']
        link_points = self.map_manager.links_data[link_id]['points']

        if start_idx >= len(link_points) - 1:
            return {'link_id': link_id, 'point_idx': len(link_points) - 1}

        link_vec = link_points[start_idx + 1][:2] - link_points[start_idx][:2]
        if np.linalg.norm(link_vec) < 1e-6:
            return {'link_id': link_id, 'point_idx': start_idx}
        link_vec_norm = link_vec / np.linalg.norm(link_vec)
        target_heading_rad = math.radians(self.target_status.heading)
        heading_vec = np.array([math.cos(target_heading_rad), math.sin(target_heading_rad)])
        dot_product = np.dot(link_vec_norm, heading_vec)

        total_dist = 0
        if dot_product > 0:
            for i in range(start_idx, len(link_points) - 1):
                total_dist += np.linalg.norm(link_points[i+1][:2] - link_points[i][:2])
                if total_dist >= intercept_distance_ahead:
                    return {'link_id': link_id, 'point_idx': i + 1}
            return {'link_id': link_id, 'point_idx': len(link_points) - 1}
        else:
            if start_idx == 0:
                return {'link_id': link_id, 'point_idx': 0}
            for i in range(start_idx, 0, -1):
                total_dist += np.linalg.norm(link_points[i-1][:2] - link_points[i][:2])
                if total_dist >= intercept_distance_ahead:
                    return {'link_id': link_id, 'point_idx': i - 1}
            return {'link_id': link_id, 'point_idx': 0}

    def _should_update_global_path(self, goal_proj):
        if not self.global_path or not self.global_path.poses:
            return True
        end_point = self.global_path.poses[-1].pose.position
        end_proj = self.map_manager.find_projection_on_map(end_point.x, end_point.y)
        return not end_proj or goal_proj['link_id'] != end_proj['link_id']

    def _stitch_links_to_path(self, path_sequence, ego_proj):
        points = []
        for i, (link_id, direction) in enumerate(path_sequence):
            link_points = self.map_manager.links_data[link_id]['points']
            if direction == 'reverse': link_points = np.flip(link_points, axis=0)

            if i == 0:
                points.extend(link_points[ego_proj['point_idx']:])
            else:
                points.extend(link_points)
        return self.convert_points_to_ros_path(points)

    def _extract_local_path(self):
        if not self.global_path or not self.global_path.poses: return None
        path_points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path.poses])
        distances = np.linalg.norm(path_points - [self.pursuer_status.position.x, self.pursuer_status.position.y], axis=1)
        start_idx = np.argmin(distances)

        path_length, end_idx = 0, len(self.global_path.poses)
        for i in range(start_idx, len(self.global_path.poses) - 1):
            p1 = self.global_path.poses[i].pose.position
            p2 = self.global_path.poses[i+1].pose.position
            path_length += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            if path_length > self.LOOKAHEAD_DISTANCE:
                end_idx = i + 1
                break

        local_path = Path(header=rospy.Header(frame_id='map', stamp=rospy.Time.now()))
        local_path.poses = self.global_path.poses[start_idx:end_idx]

        if not local_path.poses:
            return None

        return local_path

    def calculate_steering(self):
        if not self.global_path or not self.global_path.poses or not self.local_path or not self.local_path.poses:
            return self.last_steering * 0.9

        current_v = self.pursuer_status.velocity.x
        lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * current_v))

        for pose in self.local_path.poses:
            dx = pose.pose.position.x - self.pursuer_status.position.x
            dy = pose.pose.position.y - self.pursuer_status.position.y
            yaw = math.radians(self.pursuer_status.heading)

            lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)

            if lx > 0.1:
                dist = math.sqrt(lx**2 + ly**2)
                if dist >= lfd:
                    theta = math.atan2(ly, lx)

                    final_goal_pose = self.global_path.poses[-1].pose.position
                    dist_to_final_goal = math.sqrt((final_goal_pose.x - self.pursuer_status.position.x)**2 +
                                                   (final_goal_pose.y - self.pursuer_status.position.y)**2)

                    if dist_to_final_goal < self.AGGRESSIVE_TURN_DISTANCE and abs(theta) > self.AGGRESSIVE_ANGLE_THRESHOLD:
                        clipped = math.copysign(self.max_steering_rad, theta)
                    else:
                        raw_steer = math.atan2(2 * self.vehicle_length * math.sin(theta), lfd)
                        clipped = max(-self.max_steering_rad, min(self.max_steering_rad, raw_steer))

                    smoothed = self.last_steering * self.steer_smoothing_alpha + clipped * (1 - self.steer_smoothing_alpha)
                    self.last_steering = smoothed
                    return smoothed

        return self.last_steering * 0.9

    def calculate_velocity(self, goal_proj):
        goal_point = self.map_manager.links_data[goal_proj['link_id']]['points'][goal_proj['point_idx']]
        dx = goal_point[0] - self.pursuer_status.position.x
        dy = goal_point[1] - self.pursuer_status.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        target_v = self.VELOCITY_KP * dist_to_goal
        max_v = self.MAX_VELOCITY_KPH / 3.6

        if dist_to_goal < self.AGGRESSIVE_TURN_DISTANCE:
            min_aggressive_v = self.MIN_AGGRESSIVE_VELOCITY_KPH / 3.6
            return max(min_aggressive_v, min(target_v, max_v))

        return max(0.0, min(target_v, max_v))

    def convert_points_to_ros_path(self, points):
        ros_path = Path(header=rospy.Header(frame_id='map', stamp=rospy.Time.now()))
        for p in points:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.position.z = p[2]
            ros_path.poses.append(pose)
        return ros_path

if __name__ == '__main__':
    try:
        node = AggressivePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("프로그램 종료")
