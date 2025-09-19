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
        - start_proj (dict): 시작점의 맵 투영 정보
        - end_proj (dict): 도착점의 맵 투영 정보
        - reverse_penalty (float): 역주행 시 부여할 페널티 가중치
        - return (list of tuples): [(link_id, direction), ...] 형태의 경로 시퀀스
        """
        start_link_id, end_link_id = start_proj['link_id'], end_proj['link_id']
        
        # open_set: 탐색할 노드(링크)를 담는 우선순위 큐. (f_score, link_id)
        open_set = [(0, start_link_id)]
        # came_from: 경로를 역추적하기 위한 딕셔너리. {다음_링크: (이전_링크, 방향)}
        came_from = {}
        # g_score: 시작점에서 각 링크까지의 실제 비용
        g_score = {link_id: float('inf') for link_id in self.links_data}; g_score[start_link_id] = 0
        # f_score: g_score + 휴리스틱. 예상되는 총 비용
        f_score = {link_id: float('inf') for link_id in self.links_data}; f_score[start_link_id] = self._heuristic(start_link_id, end_link_id)

        while open_set:
            _, current_link_id = heapq.heappop(open_set)
            if current_link_id == end_link_id: return self._reconstruct_path(came_from, current_link_id)
            
            current_link = self.links_data[current_link_id]
            to_node_id, from_node_id = current_link['to_node_idx'], current_link['from_node_idx']
            
            # 이웃 노드(연결된 링크) 탐색
            neighbors = []
            # 정방향으로 연결된 링크
            neighbors.extend([(l_id, 'forward', current_link['link_length']) for l_id, link in self.links_data.items() if link['from_node_idx'] == to_node_id])
            # 역방향으로 연결된 링크
            neighbors.extend([(l_id, 'reverse', current_link['link_length'] * reverse_penalty) for l_id, link in self.links_data.items() if link['to_node_idx'] == from_node_id])

            for neighbor_id, direction, cost in neighbors:
                if g_score[current_link_id] + cost < g_score[neighbor_id]:
                    came_from[neighbor_id] = (current_link_id, direction)
                    g_score[neighbor_id] = g_score[current_link_id] + cost
                    f_score[neighbor_id] = g_score[neighbor_id] + self._heuristic(neighbor_id, end_link_id)
                    heapq.heappush(open_set, (f_score[neighbor_id], neighbor_id))
        return None

    def _heuristic(self, link_id_a, link_id_b):
        """A* 알고리즘의 휴리스틱 함수. 두 링크 끝점 사이의 직선 거리를 계산합니다."""
        pos_a = self.links_data[link_id_a]['points'][-1]
        pos_b = self.links_data[link_id_b]['points'][-1]
        return np.linalg.norm(pos_a - pos_b)

    def _reconstruct_path(self, came_from, current_id):
        """A* 탐색이 끝난 후, came_from 딕셔너리를 이용해 최종 경로를 생성합니다."""
        path_sequence = []
        while current_id in came_from:
            prev_id, direction = came_from[current_id]
            path_sequence.insert(0, (current_id, direction))
            current_id = prev_id
        path_sequence.insert(0, (current_id, 'forward'))
        return path_sequence

class PathVisualizer:
    """OpenCV를 사용하여 맵, 차량, 경로 등을 시각화하는 클래스."""
    def __init__(self, map_manager):
        self.map_manager = map_manager
        self.SCALE = 5.0  # 월드 좌표(m)를 픽셀로 변환하기 위한 스케일
        self.VIEW_WIDTH, self.VIEW_HEIGHT = 800, 800  # 화면 창의 너비와 높이
        self.map_background = self._create_map_background() # 맵 전체를 그린 배경 이미지

    def _create_map_background(self):
        """맵 데이터를 바탕으로 전체 맵을 담은 이미지를 생성합니다."""
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
                
                # Y축 반전은 화면 좌표계(Y가 아래로 증가)와 월드 좌표계(Y가 위로 증가)를 맞추기 위함입니다.
                p1_y_f, p2_y_f = bg_height_px - p1_y, bg_height_px - p2_y
                cv2.line(background, (p1_x, p1_y_f), (p2_x, p2_y_f), (100, 100, 100), 1)
        rospy.loginfo("배경 맵 이미지 생성 완료!")
        return background

    def update(self, ego_status, target_status, global_path, local_path):
        """
        매 프레임마다 호출되어 현재 상태를 화면에 그립니다.
        - ego_status: 자율주행 차량의 상태 메시지
        - target_status: 추격 대상 차량의 상태 메시지
        - global_path: 전역 경로
        - local_path: 지역 경로
        """
        # 1. ego 차량 중심의 뷰 영역을 배경 맵에서 잘라냅니다.
        ego_cx_bg = int((ego_status.position.x - self.map_manager.min_x) * self.SCALE) + 100
        ego_cy_bg_f = self.map_background.shape[0] - (int((ego_status.position.y - self.map_manager.min_y) * self.SCALE) + 100)
        try:
            view = cv2.getRectSubPix(self.map_background, (self.VIEW_WIDTH, self.VIEW_HEIGHT), (ego_cx_bg, ego_cy_bg_f))
        except cv2.error:
            view = np.zeros((self.VIEW_HEIGHT, self.VIEW_WIDTH, 3), dtype=np.uint8)

        # 2. ego 차량의 헤딩에 맞춰 뷰를 회전시켜, 항상 차량의 앞쪽이 위로 향하게 합니다.
        M = cv2.getRotationMatrix2D((self.VIEW_WIDTH/2, self.VIEW_HEIGHT/2), -ego_status.heading - 90, 1)
        rotated_view = cv2.warpAffine(view, M, (self.VIEW_WIDTH, self.VIEW_HEIGHT))

        # 3. 회전된 뷰 위에 경로와 다른 차량들을 그립니다.
        paths_to_draw = [(global_path, (0, 255, 255), 1), (local_path, (0, 255, 0), 2)] # (경로, 색상, 두께)
        for path, color, thickness in paths_to_draw:
            if path:
                for i in range(len(path.poses) - 1):
                    p1 = self._world_to_rotated_pixel(path.poses[i].pose.position, ego_status)
                    p2 = self._world_to_rotated_pixel(path.poses[i+1].pose.position, ego_status)
                    cv2.line(rotated_view, p1, p2, color, thickness)
        
        if target_status:
            tx, ty = self._world_to_rotated_pixel(target_status.position, ego_status)
            cv2.circle(rotated_view, (tx, ty), 10, (0, 0, 255), -1) # 빨간색 원으로 목표 차량 표시
            
        # 4. ego 차량을 화면 중앙에 파란색 원으로 표시합니다.
        cv2.circle(rotated_view, (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2), 10, (255, 0, 0), -1)
        cv2.line(rotated_view, (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2), (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2 - 20), (255, 255, 0), 2)

        cv2.imshow("Hierarchical Path Planning", rotated_view)
        cv2.waitKey(1)

    def _world_to_rotated_pixel(self, pos, ego):
        """
        월드 좌표계의 점(pos)을 회전된 ego 중심 뷰의 픽셀 좌표로 변환합니다.
        - pos (geometry_msgs/Point): 변환할 점의 월드 좌표
        - ego (EgoVehicleStatus): 기준이 되는 ego 차량의 상태
        - return (tuple): (pixel_x, pixel_y)
        """
        # 1. 월드 좌표를 ego 차량 기준의 로컬 좌표로 변환 (ego의 위치가 원점)
        dx = pos.x - ego.position.x
        dy = pos.y - ego.position.y
        yaw = math.radians(ego.heading)
        
        # 2. ego의 헤딩을 기준으로 좌표를 회전 (lx: 차량의 전방, ly: 차량의 좌측)
        lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)

        # 3. 로컬 좌표를 화면 픽셀 좌표로 변환
        pixel_x = int(self.VIEW_WIDTH/2 + ly * self.SCALE)
        pixel_y = int(self.VIEW_HEIGHT/2 - lx * self.SCALE) # lx(전방)가 양수일수록 pixel_y는 작아짐(위로 이동)
        return pixel_x, pixel_y

class AggressivePursuitNode:
    """
    공격적인 추격 주행을 수행하는 메인 ROS 노드 클래스.
    """
    def __init__(self):
        rospy.init_node("aggressive_pursuit_node", anonymous=True)
        map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_data")
        self.map_manager = HDMapManager(map_dir)
        self.visualizer = PathVisualizer(self.map_manager)
        
        self.chase_active = False # 추격 활성화 상태 플래그
        
        # ROS Subscribers
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/Ego_2/Ego_topic", EgoVehicleStatus, self.target_status_callback)
        rospy.Subscriber("/chase_status", String, self.chase_status_callback, queue_size=1)
        
        # ROS Publishers
        self.g_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.l_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        
        # 차량 상태 변수
        self.ego_status, self.target_status = None, None
        self.global_path, self.local_path = None, None
        self.last_steering = 0.0 # 조향각 부드럽게 만들기 위한 이전 값
        
        # --- 주행 파라미터 ---
        self.REVERSE_DRIVING_PENALTY = 1.0
        self.INTERCEPT_TIME_GAIN = 1.2          # 충돌 예상 시간 계산에 곱해줄 가중치 (클수록 더 먼 미래를 예측)
        self.LOOKAHEAD_DISTANCE = 50.0
        self.GLOBAL_PATH_UPDATE_THRESHOLD = 15.0
        self.vehicle_length = 2.6
        self.max_steering_rad = 0.85
        self.steer_smoothing_alpha = 0.3
        self.lfd_gain = 0.8
        self.min_lfd, self.max_lfd = 4.0, 30.0
        self.VELOCITY_KP = 1.5
        self.MAX_VELOCITY_KPH = 80.0
        
        # 공격적 회전 로직을 위한 파라미터
        self.AGGRESSIVE_TURN_DISTANCE = 10.0
        self.AGGRESSIVE_ANGLE_THRESHOLD = math.radians(30)
        self.MIN_AGGRESSIVE_VELOCITY_KPH = 15.0
        
        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop) # 20Hz로 제어 루프 실행
        rospy.on_shutdown(self.shutdown_callback)
        rospy.loginfo("공격적 최단경로 추격 노드 시작.")

    def chase_status_callback(self, msg: String):
        """ChaseManager로부터 추격 상태 명령을 수신합니다."""
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

    def control_loop(self, event):
        """메인 제어 루프. 타이머에 의해 주기적으로 실행됩니다."""
        
        # 추격이 활성화되지 않았다면, 정지 명령을 보내고 로직을 종료합니다.
        if not self.chase_active:
            if self.ego_status and self.ego_status.velocity.x > 0.5: # 완전히 멈출 때까지 브레이크
                stop_cmd = CtrlCmd(longlCmdType=1, accel=0.0, brake=1.0, steering=0.0)
                self.ctrl_pub.publish(stop_cmd)
            return

        if not self.ego_status or not self.target_status: return

        # 1. 현재 차량 위치를 맵에 투영
        ego_proj = self.map_manager.find_projection_on_map(self.ego_status.position.x, self.ego_status.position.y)
        target_proj = self.map_manager.find_projection_on_map(self.target_status.position.x, self.target_status.position.y)
        if not ego_proj or not target_proj: return

        # 2. 목표 지점(충돌 위치) 계산
        goal_proj = self._get_intercept_goal_projection(target_proj)

        # 3. 전역 경로 생성 또는 업데이트
        if self.global_path is None or self._should_update_global_path(goal_proj):
            rospy.loginfo("전역 경로를 재탐색합니다...")
            path_seq = self.map_manager.get_global_path(ego_proj, goal_proj, self.REVERSE_DRIVING_PENALTY)
            if path_seq:
                self.global_path = self._stitch_links_to_path(path_seq, ego_proj)
                self.g_path_pub.publish(self.global_path)
            else: self.global_path = None
        
        if not self.global_path: return

        # 4. 지역 경로 추출
        self.local_path = self._extract_local_path()
        if not self.local_path: return
        self.l_path_pub.publish(self.local_path)

        # 5. 조향 및 속도 계산
        steering_cmd = self.calculate_steering()
        velocity_cmd = self.calculate_velocity(goal_proj)

        # 6. 차량 제어 명령 전송
        cmd = CtrlCmd(longlCmdType=2, steering=float(steering_cmd), velocity=float(velocity_cmd))
        self.ctrl_pub.publish(cmd)

        # 7. 시각화 업데이트
        self.visualizer.update(self.ego_status, self.target_status, self.global_path, self.local_path)

    def _get_intercept_goal_projection(self, target_proj):
        """
        목표 차량의 미래 위치를 예측하여 '충돌 지점'을 목표로 반환합니다.
        """
        # 1. 두 차량 간의 직선 거리를 기반으로 예상 소요 시간(time_to_intercept) 계산
        dx = self.target_status.position.x - self.ego_status.position.x
        dy = self.target_status.position.y - self.ego_status.position.y
        dist_direct = math.sqrt(dx**2 + dy**2)

        ego_speed = self.ego_status.velocity.x
        if ego_speed < 1.0: # 속도가 0에 가까울 때를 대비한 예외처리
            ego_speed = 1.0
            
        time_to_intercept = (dist_direct / ego_speed) * self.INTERCEPT_TIME_GAIN

        # 2. 예상 소요 시간 동안 목표 차량이 이동할 거리 계산
        target_speed = self.target_status.velocity.x
        intercept_distance_ahead = target_speed * time_to_intercept

        # 3. 목표 차량의 현재 위치에서 '예상 이동 거리'만큼 떨어진 지점을 맵 상에서 탐색
        link_id = target_proj['link_id']
        start_idx = target_proj['point_idx']
        link_points = self.map_manager.links_data[link_id]['points']

        if start_idx >= len(link_points) - 1:
            return {'link_id': link_id, 'point_idx': len(link_points) - 1}

        # 목표 차량의 주행 방향 판별
        link_vec = link_points[start_idx + 1][:2] - link_points[start_idx][:2]
        if np.linalg.norm(link_vec) < 1e-6:
            return {'link_id': link_id, 'point_idx': start_idx}
        link_vec_norm = link_vec / np.linalg.norm(link_vec)
        target_heading_rad = math.radians(self.target_status.heading)
        heading_vec = np.array([math.cos(target_heading_rad), math.sin(target_heading_rad)])
        dot_product = np.dot(link_vec_norm, heading_vec)

        # 계산된 intercept_distance_ahead를 사용하여 목표점 탐색
        total_dist = 0
        if dot_product > 0:  # 정방향 주행
            for i in range(start_idx, len(link_points) - 1):
                total_dist += np.linalg.norm(link_points[i+1][:2] - link_points[i][:2])
                if total_dist >= intercept_distance_ahead:
                    return {'link_id': link_id, 'point_idx': i + 1}
            return {'link_id': link_id, 'point_idx': len(link_points) - 1}
        else:  # 역방향 주행
            if start_idx == 0:
                return {'link_id': link_id, 'point_idx': 0}
            for i in range(start_idx, 0, -1):
                total_dist += np.linalg.norm(link_points[i-1][:2] - link_points[i][:2])
                if total_dist >= intercept_distance_ahead:
                    return {'link_id': link_id, 'point_idx': i - 1}
            return {'link_id': link_id, 'point_idx': 0}

    def _should_update_global_path(self, goal_proj):
        """전역 경로를 다시 탐색해야 하는지 결정합니다."""
        if not self.global_path or not self.global_path.poses: 
            return True
        end_point = self.global_path.poses[-1].pose.position
        end_proj = self.map_manager.find_projection_on_map(end_point.x, end_point.y)
        return not end_proj or goal_proj['link_id'] != end_proj['link_id']

    def _stitch_links_to_path(self, path_sequence, ego_proj):
        """A*로 찾은 링크 시퀀스를 하나의 연속된 포인트 리스트로 만듭니다."""
        points = []
        for i, (link_id, direction) in enumerate(path_sequence):
            link_points = self.map_manager.links_data[link_id]['points']
            if direction == 'reverse': link_points = np.flip(link_points, axis=0)
            
            if i == 0: # 첫 번째 링크는 ego의 현재 위치부터 시작
                points.extend(link_points[ego_proj['point_idx']:])
            else:
                points.extend(link_points)
        return self.convert_points_to_ros_path(points)
        
    def _extract_local_path(self):
        """전체 전역 경로에서 차량이 당장 따라가야 할 지역 경로를 추출합니다."""
        if not self.global_path or not self.global_path.poses: return None
        # 전역 경로의 모든 점들을 Numpy 배열로 변환
        path_points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path.poses])
        # 차량과 가장 가까운 경로상의 점을 찾기
        distances = np.linalg.norm(path_points - [self.ego_status.position.x, self.ego_status.position.y], axis=1)
        start_idx = np.argmin(distances)
        
        # 가장 가까운 점에서부터 LOOKAHEAD_DISTANCE 만큼의 경로를 추출
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
        """Pure Pursuit 알고리즘을 사용하여 목표 조향각을 계산합니다."""
        if not self.global_path or not self.global_path.poses or not self.local_path or not self.local_path.poses:
            return self.last_steering * 0.9
            
        current_v = self.ego_status.velocity.x
        # 속도에 비례하여 전방 주시 거리(lfd)를 동적으로 계산
        lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * current_v))
        
        for pose in self.local_path.poses:
            dx = pose.pose.position.x - self.ego_status.position.x
            dy = pose.pose.position.y - self.ego_status.position.y
            yaw = math.radians(self.ego_status.heading)
            # 목표점을 차량의 로컬 좌표계로 변환 (lx: 전방, ly: 좌측)
            lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
            
            if lx > 0.1: # 차량 앞쪽에 있는 점만 고려
                dist = math.sqrt(lx**2 + ly**2)
                if dist >= lfd: # 전방 주시 거리(lfd)보다 멀리 있는 첫 번째 점을 목표점으로 설정
                    # theta: 차량의 현재 방향과 목표점 사이의 각도
                    theta = math.atan2(ly, lx)
                    
                    # 최종 목표 지점과의 거리를 계산
                    final_goal_pose = self.global_path.poses[-1].pose.position
                    dist_to_final_goal = math.sqrt((final_goal_pose.x - self.ego_status.position.x)**2 + 
                                                   (final_goal_pose.y - self.ego_status.position.y)**2)

                    # 공격적 회전 조건: 목표 지점에 가깝고, 회전각이 클 때
                    if dist_to_final_goal < self.AGGRESSIVE_TURN_DISTANCE and abs(theta) > self.AGGRESSIVE_ANGLE_THRESHOLD:
                        clipped = math.copysign(self.max_steering_rad, theta) # 최대 조향각 사용
                    else:
                        # 일반적인 Pure Pursuit 조향각 계산
                        raw_steer = math.atan2(2 * self.vehicle_length * math.sin(theta), lfd)
                        clipped = max(-self.max_steering_rad, min(self.max_steering_rad, raw_steer))
                    
                    # 조향각을 부드럽게 만들기 (Low-pass filter)
                    smoothed = self.last_steering * self.steer_smoothing_alpha + clipped * (1 - self.steer_smoothing_alpha)
                    self.last_steering = smoothed
                    return smoothed
                    
        return self.last_steering * 0.9

    def calculate_velocity(self, goal_proj):
        """목표 지점까지의 거리에 비례하여 목표 속도를 계산합니다."""
        goal_point = self.map_manager.links_data[goal_proj['link_id']]['points'][goal_proj['point_idx']]
        dx = goal_point[0] - self.ego_status.position.x
        dy = goal_point[1] - self.ego_status.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)
        # 거리에 비례한 속도 계산 (P 제어)
        target_v = self.VELOCITY_KP * dist_to_goal
        max_v = self.MAX_VELOCITY_KPH / 3.6

        # 공격적 회전 조건: 목표 지점에 가까울 때 최소 속도 보장
        if dist_to_goal < self.AGGRESSIVE_TURN_DISTANCE:
            min_aggressive_v = self.MIN_AGGRESSIVE_VELOCITY_KPH / 3.6
            return max(min_aggressive_v, min(target_v, max_v))
            
        return max(0.0, min(target_v, max_v))

    def convert_points_to_ros_path(self, points):
        """Numpy 포인트 리스트를 ROS Path 메시지로 변환합니다."""
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
