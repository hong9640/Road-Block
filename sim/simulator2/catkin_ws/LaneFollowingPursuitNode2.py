#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import math
import os
import numpy as np
from scipy.spatial import KDTree
import heapq  # A* 알고리즘을 위한 우선순위 큐
import cv2

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

# -------------------------------------------------------------------
# HDMapManager: A* 경로 탐색 기능이 추가되어 대폭 강화됨
# -------------------------------------------------------------------
class HDMapManager:
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
                self.max_x, self.max_y = max(self.max_x, x), max(self.max_y, y)
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

    def get_global_path(self, start_proj, end_proj):
        """A* 알고리즘으로 시작점에서 목적지까지의 최적 링크 시퀀스를 찾음"""
        start_link_id = start_proj['link_id']
        end_link_id = end_proj['link_id']

        # A* 알고리즘 초기화
        open_set = [(0, start_link_id)]  # (f_score, link_id)
        came_from = {}
        g_score = {link_id: float('inf') for link_id in self.links_data}
        g_score[start_link_id] = 0
        f_score = {link_id: float('inf') for link_id in self.links_data}
        f_score[start_link_id] = self._heuristic(start_link_id, end_link_id)

        while open_set:
            _, current_link_id = heapq.heappop(open_set)

            if current_link_id == end_link_id:
                return self._reconstruct_path(came_from, current_link_id)

            current_link = self.links_data[current_link_id]
            
            # 이웃 탐색 (다음 링크들)
            to_node_id = current_link['to_node_idx']
            neighbors = [l_id for l_id, link in self.links_data.items() if link['from_node_idx'] == to_node_id]

            for neighbor_link_id in neighbors:
                tentative_g_score = g_score[current_link_id] + self.links_data[neighbor_link_id]['link_length']
                if tentative_g_score < g_score[neighbor_link_id]:
                    came_from[neighbor_link_id] = current_link_id
                    g_score[neighbor_link_id] = tentative_g_score
                    f_score[neighbor_link_id] = tentative_g_score + self._heuristic(neighbor_link_id, end_link_id)
                    if (f_score[neighbor_link_id], neighbor_link_id) not in open_set:
                        heapq.heappush(open_set, (f_score[neighbor_link_id], neighbor_link_id))
        return None # 경로를 찾지 못함

    def _heuristic(self, link_id_a, link_id_b):
        """A* 휴리스틱 함수 (두 링크의 끝점 사이의 유클리드 거리)"""
        pos_a = self.links_data[link_id_a]['points'][-1]
        pos_b = self.links_data[link_id_b]['points'][-1]
        return np.linalg.norm(pos_a - pos_b)

    def _reconstruct_path(self, came_from, current_id):
        """A* 결과로부터 경로(링크 ID 시퀀스)를 재구성"""
        total_path = [current_id]
        while current_id in came_from:
            current_id = came_from[current_id]
            total_path.insert(0, current_id)
        return total_path

# -------------------------------------------------------------------
# PathVisualizer는 이전과 동일 (배경 미리 그리기 최적화 버전)
# 단, Global Path를 그리는 기능 추가
# -------------------------------------------------------------------
class PathVisualizer:
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
        
        for link_id, link in self.map_manager.links_data.items():
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

        # 전역 경로 그리기 (노란색)
        if global_path is not None:
            for i in range(len(global_path.poses) - 1):
                p1 = self._world_to_rotated_pixel(global_path.poses[i].pose.position, ego_status)
                p2 = self._world_to_rotated_pixel(global_path.poses[i+1].pose.position, ego_status)
                cv2.line(rotated_view, p1, p2, (0, 255, 255), 1) # Yellow

        # 지역 경로 그리기 (녹색)
        if local_path is not None:
            for i in range(len(local_path.poses) - 1):
                p1 = self._world_to_rotated_pixel(local_path.poses[i].pose.position, ego_status)
                p2 = self._world_to_rotated_pixel(local_path.poses[i+1].pose.position, ego_status)
                cv2.line(rotated_view, p1, p2, (0, 255, 0), 2) # Green
        
        if target_status is not None:
            tx, ty = self._world_to_rotated_pixel(target_status.position, ego_status)
            cv2.circle(rotated_view, (tx, ty), 10, (0, 0, 255), -1) # Red
            
        cv2.circle(rotated_view, (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2), 10, (255, 0, 0), -1) # Blue
        cv2.line(rotated_view, (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2), (self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2 - 20), (255, 255, 0), 2)

        cv2.imshow("Hierarchical Path Planning", rotated_view)
        cv2.waitKey(1)

    def _world_to_rotated_pixel(self, pos, ego):
        dx, dy = pos.x - ego.position.x, pos.y - ego.position.y
        yaw = math.radians(ego.heading)
        lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        return int(self.VIEW_WIDTH/2 + ly * self.SCALE), int(self.VIEW_HEIGHT/2 - lx * self.SCALE)

# -------------------------------------------------------------------
# 메인 노드: 계층적 경로 계획을 수행하도록 재설계
# -------------------------------------------------------------------
class HierarchicalPursuitNode:
    def __init__(self):
        rospy.init_node("hierarchical_pursuit_node", anonymous=True)
        
        map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map_data")
        self.map_manager = HDMapManager(map_dir)
        self.visualizer = PathVisualizer(self.map_manager)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/Ego-2/Ego_topic", EgoVehicleStatus, self.target_status_callback)

        self.g_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.l_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)

        self.ego_status = None
        self.target_status = None
        self.global_path_links = None
        self.global_path = None
        self.local_path = None
        self.last_steering = 0.0

        # === 튜닝 파라미터 ===
        self.LOOKAHEAD_DISTANCE = 50.0  # 지역 경로 생성 거리
        self.TARGET_DISTANCE = 15.0     # 목표 추종 거리
        self.GLOBAL_PATH_UPDATE_THRESHOLD = 10.0 # 타겟이 이 거리 이상 벗어나면 전역 경로 재탐색

        self.vehicle_length = 2.6
        self.max_steering_rad = 0.85
        self.steer_smoothing_alpha = 0.4
        self.lfd_gain = 0.9
        self.min_lfd = 4.0
        self.max_lfd = 30.0
        self.VELOCITY_KP = 1.2
        self.MAX_VELOCITY_KPH = 60.0

        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop)
        rospy.on_shutdown(self.shutdown_callback)
        rospy.loginfo("계층적 경로 계획 추격 노드 시작.")

    def shutdown_callback(self):
        rospy.loginfo("노드 종료. OpenCV 창 닫기.")
        cv2.destroyAllWindows()

    def ego_status_callback(self, msg): self.ego_status = msg
    def target_status_callback(self, msg): self.target_status = msg

    def control_loop(self, event):
        if self.ego_status is None or self.target_status is None:
            return
        
        # 1. 전역 경로(Global Path) 관리
        ego_proj = self.map_manager.find_projection_on_map(self.ego_status.position.x, self.ego_status.position.y)
        target_proj = self.map_manager.find_projection_on_map(self.target_status.position.x, self.target_status.position.y)
        if ego_proj is None or target_proj is None: return

        # 전역 경로가 없거나, 타겟이 경로 끝에서 너무 멀어지면 재탐색
        if self.global_path is None or self._should_update_global_path(target_proj):
            rospy.loginfo("전역 경로를 재탐색합니다...")
            self.global_path_links = self.map_manager.get_global_path(ego_proj, target_proj)
            if self.global_path_links:
                self.global_path = self._stitch_links_to_path(self.global_path_links)
                self.g_path_pub.publish(self.global_path)
            else:
                rospy.logwarn("전역 경로 탐색 실패.")
                self.global_path = None

        if self.global_path is None: return

        # 2. 지역 경로(Local Path) 생성
        self.local_path = self._extract_local_path()
        if self.local_path is None: return
        self.l_path_pub.publish(self.local_path)

        # 3. 제어 명령 계산 및 발행
        steering_cmd = self.calculate_steering()
        velocity_cmd = self.calculate_velocity()

        cmd = CtrlCmd(longlCmdType=2, steering=float(steering_cmd), velocity=float(velocity_cmd))
        self.ctrl_pub.publish(cmd)
        
        # 4. 시각화 업데이트
        self.visualizer.update(self.ego_status, self.target_status, self.global_path, self.local_path)

    def _should_update_global_path(self, target_proj):
        # 타겟이 현재 전역 경로의 마지막 링크와 다른지 확인
        if not self.global_path_links: return True
        
        if target_proj['link_id'] != self.global_path_links[-1]:
            # 타겟이 차선을 바꿨다면, 즉시 경로 재탐색
            return True
        
        # 타겟이 경로 끝점에서 일정 거리 이상 벗어났는지 확인 (예: 유턴 등)
        end_point = self.global_path.poses[-1].pose.position
        dist = math.sqrt((end_point.x - self.target_status.position.x)**2 + (end_point.y - self.target_status.position.y)**2)
        return dist > self.GLOBAL_PATH_UPDATE_THRESHOLD

    def _stitch_links_to_path(self, link_ids):
        points = []
        for i, link_id in enumerate(link_ids):
            link_points = self.map_manager.links_data[link_id]['points']
            if i == 0: # 시작 링크는 현재 위치부터
                start_proj = self.map_manager.find_projection_on_map(self.ego_status.position.x, self.ego_status.position.y)
                points.extend(link_points[start_proj['point_idx']:])
            else:
                points.extend(link_points)
        return self.convert_points_to_ros_path(points)
        
    def _extract_local_path(self):
        if self.global_path is None: return None
        
        ego_x, ego_y = self.ego_status.position.x, self.ego_status.position.y
        path_points = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path.poses])
        
        distances = np.linalg.norm(path_points - [ego_x, ego_y], axis=1)
        start_idx = np.argmin(distances)
        
        path_length = 0
        end_idx = start_idx
        for i in range(start_idx, len(self.global_path.poses) - 1):
            p1 = self.global_path.poses[i].pose.position
            p2 = self.global_path.poses[i+1].pose.position
            path_length += math.sqrt((p2.x-p1.x)**2 + (p2.y-p1.y)**2)
            if path_length > self.LOOKAHEAD_DISTANCE:
                end_idx = i + 1
                break
        else:
            end_idx = len(self.global_path.poses)

        local_path_msg = Path()
        local_path_msg.header.frame_id = 'map'
        local_path_msg.header.stamp = rospy.Time.now()
        local_path_msg.poses = self.global_path.poses[start_idx:end_idx]
        return local_path_msg
        
    def calculate_steering(self):
        # ... 이전과 동일 ...
        if self.local_path is None or len(self.local_path.poses) < 2: return self.last_steering * 0.9
        current_v = self.ego_status.velocity.x
        lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * current_v))
        for pose in self.local_path.poses:
            dx, dy = pose.pose.position.x - self.ego_status.position.x, pose.pose.position.y - self.ego_status.position.y
            yaw = math.radians(self.ego_status.heading)
            lx, ly = dx*math.cos(-yaw) - dy*math.sin(-yaw), dx*math.sin(-yaw) + dy*math.cos(-yaw)
            if lx > 0:
                dist = math.sqrt(lx**2+ly**2)
                if dist >= lfd:
                    theta = math.atan2(ly, lx)
                    raw_steer = math.atan2(2*self.vehicle_length*math.sin(theta), lfd)
                    clipped = max(-self.max_steering_rad, min(self.max_steering_rad, raw_steer))
                    smoothed = self.last_steering*self.steer_smoothing_alpha + clipped*(1-self.steer_smoothing_alpha)
                    self.last_steering = smoothed
                    return smoothed
        return self.last_steering * 0.9

    def calculate_velocity(self):
        # ... 이전과 동일 ...
        dx, dy = self.target_status.position.x - self.ego_status.position.x, self.target_status.position.y - self.ego_status.position.y
        dist_error = math.sqrt(dx**2+dy**2) - self.TARGET_DISTANCE
        target_v = self.VELOCITY_KP * dist_error
        max_v = self.MAX_VELOCITY_KPH / 3.6
        return max(0.0, min(target_v, max_v))

    def convert_points_to_ros_path(self, points):
        ros_path = Path(header=rospy.Header(frame_id='map', stamp=rospy.Time.now()))
        for p in points:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = p[0], p[1], p[2]
            ros_path.poses.append(pose)
        return ros_path

if __name__ == '__main__':
    try:
        node = HierarchicalPursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("프로그램 종료")
