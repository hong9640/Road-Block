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
from std_msgs.msg import String

class HDMapManager:
    """
    HD Map 데이터를 로드하고 경로 탐색에 필요한 기능을 제공하는 클래스.
    - link_set.json: 도로(링크) 정보
    - node_set.json: 교차로(노드) 정보
    """
    def __init__(self, map_dir_path):
        # links_data: 링크의 상세 정보 (포인트, 길이, 속도 등)를 담는 딕셔너리. Key: 링크 ID
        self.links_data = {}
        # nodes_data: 노드의 상세 정보 (좌표, 연결된 링크 등)를 담는 딕셔너리. Key: 노드 ID
        self.nodes_data = {}
        # waypoints_xy: 맵의 모든 링크 포인트를 [x, y] 형태로 저장하는 Numpy 배열
        self.waypoints_xy = None
        # kdtree: 모든 웨이포인트에 대한 KDTree. 가장 가까운 점을 빠르게 찾기 위해 사용.
        self.kdtree = None
        # link_info_at_waypoint: 각 웨이포인트가 어떤 링크의 몇 번째 점인지 정보를 저장하는 리스트.
        self.link_info_at_waypoint = []
        # 맵의 경계 좌표 (min/max x, y)
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
                self.max_x, self.max_y = max(self.max_x, x), max(self.max_y, y)
                all_points_xy.append([x, y])
                self.link_info_at_waypoint.append({'link_id': link['idx'], 'point_idx': i})

        if all_points_xy:
            self.waypoints_xy = np.array(all_points_xy)
            self.kdtree = KDTree(self.waypoints_xy)
            rospy.loginfo("맵 데이터 로딩 및 KDTree 생성 완료!")

    def find_projection_on_map(self, x, y):
        """
        주어진 (x, y) 좌표에서 가장 가까운 맵 상의 포인트를 찾습니다.
        - x (float): 월드 좌표계 x
        - y (float): 월드 좌표계 y
        - return (dict): {'link_id': str, 'point_idx': int} 형태의 정보
        """
        if self.kdtree is None: return None
        _, idx = self.kdtree.query([x, y], k=1)
        return self.link_info_at_waypoint[idx]

    def get_global_path(self, start_proj, end_proj, reverse_penalty=1.0):
        """
        A* 알고리즘을 사용하여 두 링크 사이의 최단 경로(링크 시퀀스)를 찾습니다.
        """
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
        path_sequence = [(current_id, 'forward')]
        while current_id in came_from:
            prev_id, direction = came_from[current_id]
            path_sequence.insert(0, (prev_id, direction))
            current_id = prev_id
        return path_sequence

class PathVisualizer:
    """OpenCV를 사용하여 맵, 차량, 경로 등을 시각화하는 클래스."""
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
        
        self.node_name = rospy.get_name()
        
        map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_data")
        self.map_manager = HDMapManager(map_dir)
        self.visualizer = PathVisualizer(self.map_manager)
        
        self.chase_active = False
        
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/Ego_2/Ego_topic", EgoVehicleStatus, self.target_status_callback)
        rospy.Subscriber("/chase_status", String, self.chase_status_callback, queue_size=1)
        
        self.g_path_pub = rospy.Publisher('global_path', Path, queue_size=1)
        self.l_path_pub = rospy.Publisher('local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('ctrl_cmd', CtrlCmd, queue_size=1)
        
        self.ego_status, self.target_status = None, None
        self.global_path, self.local_path = None, None
        self.last_steering = 0.0
        
        # --- 주행 파라미터 (고속 안정성 튜닝 적용) ---
        self.REVERSE_DRIVING_PENALTY = 1.0
        self.INTERCEPT_TIME_GAIN = 1.2
        self.LOOKAHEAD_DISTANCE = 50.0
        self.GLOBAL_PATH_UPDATE_THRESHOLD = 15.0
        self.vehicle_length = 2.6
        self.max_steering_rad = 0.85
        self.steer_smoothing_alpha = 0.3
        
        # 고속 안정성을 위한 튜닝 파라미터
        self.lfd_gain = 1.2                  # 속도 대비 전방 주시 거리 게인 (증가)
        self.min_lfd, self.max_lfd = 4.0, 60.0 # 최대 전방 주시 거리 (증가)
        self.VELOCITY_KP = 1.0               # 속도 비례 상수 (감소시켜 부드러운 가속)
        self.MAX_VELOCITY_KPH = 250.0         # 최고 속도
        
        # 공격적 회전 로직을 위한 파라미터
        self.AGGRESSIVE_TURN_DISTANCE = 10.0
        self.AGGRESSIVE_ANGLE_THRESHOLD = math.radians(30)
        self.MIN_AGGRESSIVE_VELOCITY_KPH = 15.0
        
        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop)
        rospy.on_shutdown(self.shutdown_callback)
        #rospy.loginfo("공격적 최단경로 추격 노드 시작.")

    def chase_status_callback(self, msg: String):
        rospy.loginfo(f"추격 상태 명령 수신: '{msg.data}'")
        if msg.data == "START" and not self.chase_active:
            self.chase_active = True
            rospy.loginfo("추격 명령 확인! 주행을 시작합니다.")
        elif msg.data == "STOP" and self.chase_active:
            self.chase_active = False
            rospy.loginfo("정지 명령 확인! 주행을 중지합니다.")

    def shutdown_callback(self):
        rospy.loginfo("노드 종료. OpenCV 창 닫기.")
        cv2.destroyAllWindows()

    def ego_status_callback(self, msg): self.ego_status = msg
    def target_status_callback(self, msg): self.target_status = msg

    def _determine_driving_mode(self):
        """Ego 차량의 헤딩과 목표 지점의 방향을 비교하여 전진/후진 모드를 결정합니다."""
        ego_yaw_rad = math.radians(self.ego_status.heading)
        ego_heading_vec = np.array([math.cos(ego_yaw_rad), math.sin(ego_yaw_rad)])
        
        target_vec = np.array([self.target_status.position.x - self.ego_status.position.x,
                               self.target_status.position.y - self.ego_status.position.y])
        
        if np.linalg.norm(target_vec) < 1e-6:
            return 'forward'

        target_vec_norm = target_vec / np.linalg.norm(target_vec)
        dot_product = np.dot(ego_heading_vec, target_vec_norm)
        
        if dot_product < -0.2:
            return 'reverse'
        else:
            return 'forward'

    def control_loop(self, event):
        """메인 제어 루프. 타이머에 의해 주기적으로 실행됩니다."""
        if not self.chase_active:
            if self.ego_status and self.ego_status.velocity.x > 0.5:
                stop_cmd = CtrlCmd(longlCmdType=1, accel=0.0, brake=1.0, steering=0.0)
                self.ctrl_pub.publish(stop_cmd)
            return

        if not self.ego_status or not self.target_status: return

        ego_proj = self.map_manager.find_projection_on_map(self.ego_status.position.x, self.ego_status.position.y)
        target_proj = self.map_manager.find_projection_on_map(self.target_status.position.x, self.target_status.position.y)
        if not ego_proj or not target_proj: return

        goal_proj = self._get_intercept_goal_projection(target_proj)
        
        ego_link_id = ego_proj['link_id']
        goal_link_id = goal_proj['link_id']

        should_replan = False
        if self.global_path is None:
            should_replan = True
        else:
            last_pose = self.global_path.poses[-1].pose.position
            last_proj = self.map_manager.find_projection_on_map(last_pose.x, last_pose.y)
            if not last_proj or last_proj['link_id'] != goal_link_id:
                should_replan = True
        
        if should_replan:
            # 상황 1: 목표가 같은 도로에 있을 때 (직선 경로 생성)
            if ego_link_id == goal_link_id:
                #rospy.loginfo("목표가 동일 링크 상에 있음. 직선 경로를 생성합니다.")
                link_points = self.map_manager.links_data[ego_link_id]['points']
                
                start_idx = ego_proj['point_idx']
                end_idx = goal_proj['point_idx']
                
                path_points = []
                if start_idx <= end_idx:
                    path_points = link_points[start_idx : end_idx + 1]
                else:
                    path_points = np.flip(link_points[end_idx : start_idx + 1], axis=0)

                if len(path_points) > 0:
                    self.global_path = self.convert_points_to_ros_path(path_points)
                    self.g_path_pub.publish(self.global_path)
                else:
                    self.global_path = None

            # 상황 2: 목표가 다른 도로에 있을 때 (A* 탐색)
            else:
                #rospy.loginfo("목표가 다른 링크 상에 있음. A* 경로 탐색을 시작합니다.")
                driving_mode = self._determine_driving_mode()
                path_seq = None

                if driving_mode == 'reverse':
                    path_seq = self.map_manager.get_global_path(goal_proj, ego_proj, self.REVERSE_DRIVING_PENALTY)
                else:
                    path_seq = self.map_manager.get_global_path(ego_proj, goal_proj, self.REVERSE_DRIVING_PENALTY)

                if path_seq:
                    start_proj_for_stitch = goal_proj if driving_mode == 'reverse' else ego_proj
                    path_msg = self._stitch_links_to_path(path_seq, start_proj_for_stitch)
                    
                    if driving_mode == 'reverse':
                        path_msg.poses.reverse()

                    self.global_path = path_msg
                    self.g_path_pub.publish(self.global_path)
                else:
                    self.global_path = None
        
        if not self.global_path: return

        self.local_path = self._extract_local_path()
        if not self.local_path: return
        self.l_path_pub.publish(self.local_path)

        steering_driving_mode = self._determine_driving_mode()
        steering_cmd = self.calculate_steering(steering_driving_mode)
        velocity_cmd = self.calculate_velocity(goal_proj)

        cmd = CtrlCmd(longlCmdType=2, steering=float(steering_cmd), velocity=float(velocity_cmd))
        self.ctrl_pub.publish(cmd)

        self.visualizer.update(self.ego_status, self.target_status, self.global_path, self.local_path)

    def calculate_steering(self, driving_mode='forward'):
        """Pure Pursuit 알고리즘을 사용하여 목표 조향각을 계산합니다."""
        if not self.local_path or not self.local_path.poses:
            return self.last_steering * 0.9
            
        current_v = self.ego_status.velocity.x
        lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * current_v))
        
        for pose in self.local_path.poses:
            dx = pose.pose.position.x - self.ego_status.position.x
            dy = pose.pose.position.y - self.ego_status.position.y
            yaw = math.radians(self.ego_status.heading)
            
            lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
            
            is_lookahead_point = False
            if driving_mode == 'reverse':
                if lx < -0.1: 
                    dist = math.sqrt(lx**2 + ly**2)
                    if dist >= lfd:
                        is_lookahead_point = True
            else:
                if lx > 0.1:
                    dist = math.sqrt(lx**2 + ly**2)
                    if dist >= lfd:
                        is_lookahead_point = True
            
            if is_lookahead_point:
                theta = math.atan2(ly, lx)
                
                final_goal_pose = self.global_path.poses[-1].pose.position
                dist_to_final_goal = math.sqrt((final_goal_pose.x - self.ego_status.position.x)**2 + 
                                               (final_goal_pose.y - self.ego_status.position.y)**2)
                
                if dist_to_final_goal < self.AGGRESSIVE_TURN_DISTANCE and abs(theta) > self.AGGRESSIVE_ANGLE_THRESHOLD:
                    clipped = math.copysign(self.max_steering_rad, theta)
                else:
                    raw_steer = math.atan2(2 * self.vehicle_length * math.sin(theta), lfd)
                    clipped = max(-self.max_steering_rad, min(self.max_steering_rad, raw_steer))
                
                if driving_mode == 'reverse':
                    clipped = -clipped
                
                smoothed = self.last_steering * self.steer_smoothing_alpha + clipped * (1 - self.steer_smoothing_alpha)
                self.last_steering = smoothed
                return smoothed
                                
        return self.last_steering * 0.9

    def _get_intercept_goal_projection(self, target_proj):
        dx = self.target_status.position.x - self.ego_status.position.x
        dy = self.target_status.position.y - self.ego_status.position.y
        dist_direct = math.sqrt(dx**2 + dy**2)

        ego_speed = self.ego_status.velocity.x
        if ego_speed < 1.0: ego_speed = 1.0
            
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

    def _stitch_links_to_path(self, path_sequence, start_proj):
        points = []
        for i, (link_id, direction) in enumerate(path_sequence):
            link_points_orig = self.map_manager.links_data[link_id]['points']
            
            if direction == 'reverse': 
                link_points = np.flip(link_points_orig, axis=0)
            else:
                link_points = link_points_orig
            
            if i == 0:
                start_point_idx = start_proj['point_idx']
                points.extend(link_points[start_point_idx:])
            else:
                points.extend(link_points)
                
        return self.convert_points_to_ros_path(points)
        
    def _extract_local_path(self):
        if not self.global_path or not self.global_path.poses: return None
        path_points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path.poses])
        distances = np.linalg.norm(path_points - [self.ego_status.position.x, self.ego_status.position.y], axis=1)
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

        return local_path if local_path.poses else None
        
    def calculate_velocity(self, goal_proj):
        goal_point = self.map_manager.links_data[goal_proj['link_id']]['points'][goal_proj['point_idx']]
        dx = goal_point[0] - self.ego_status.position.x
        dy = goal_point[1] - self.ego_status.position.y
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
            pose.pose.position.z = p[2] if len(p) > 2 else 0.0
            ros_path.poses.append(pose)
        return ros_path

if __name__ == '__main__':
    try:
        node = AggressivePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("프로그램 종료")
