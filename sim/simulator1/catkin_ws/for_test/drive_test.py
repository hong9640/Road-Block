#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: drive_test.py  (km/h 퍼블리시 수정 버전)

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

# LiDAR
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2

# ==============================
#           MAP LAYER
# ==============================
class HDMapManager:
    # HD 맵 로딩 및 KDTree 생성
    def __init__(self, map_dir_path):
        self.links_data = {}
        self.nodes_data = {}
        self.waypoints_xy = None
        self.kdtree = None
        self.link_info_at_waypoint = []
        self.min_x, self.min_y = float('inf'), float('inf')
        self.max_x, self.max_y = float('-inf'), float('-inf')

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

        for node in node_data:
            self.nodes_data[node['idx']] = node

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

    def get_global_path(self, start_proj, end_proj, reverse_penalty=1.0):
        start_link_id, end_link_id = start_proj['link_id'], end_proj['link_id']

        open_set = [(0, start_link_id)]
        came_from = {}
        g_score = {link_id: float('inf') for link_id in self.links_data}; g_score[start_link_id] = 0
        f_score = {link_id: float('inf') for link_id in self.links_data}; f_score[start_link_id] = self._heuristic(start_link_id, end_link_id)

        while open_set:
            _, current_link_id = heapq.heappop(open_set)
            if current_link_id == end_link_id:
                return self._reconstruct_path(came_from, current_link_id)

            current_link = self.links_data[current_link_id]
            to_node_id, from_node_id = current_link['to_node_idx'], current_link['from_node_idx']

            neighbors = []
            neighbors.extend([(l_id, 'forward', current_link['link_length'])
                              for l_id, link in self.links_data.items()
                              if link['from_node_idx'] == to_node_id])
            neighbors.extend([(l_id, 'reverse', current_link['link_length'] * reverse_penalty)
                              for l_id, link in self.links_data.items()
                              if link['to_node_idx'] == from_node_id])

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

# ==============================
#        VISUALIZATION
# ==============================
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

# ==============================
#     LIDAR OBSTACLE MODULE
# ==============================
class LiDARObstacleModule:
    """
    - 전방 ROI 내 포인트로 전방 점유/빈틈 판단
    - 차량 차폭 + 여유폭 기준으로 통과 가능한 gap 중앙각 추정
    - TTC 재료(전방 최소거리) 제공
    """
    def __init__(self, points_topic=None):
        self.points_topic = points_topic if points_topic is not None else rospy.get_param("~points_topic", "/points_raw")
        self.lidar_in_base_link = rospy.get_param("~lidar_in_base_link", True)

        self.roi_x = rospy.get_param("~lidar_roi_x", [0.5, 30.0])
        self.roi_y = rospy.get_param("~lidar_roi_y", [-5.0, 5.0])
        self.roi_z = rospy.get_param("~lidar_roi_z", [-1.5, 2.0])

        self.fov_deg = rospy.get_param("~lidar_fov_deg", 120.0)
        self.num_bins = rospy.get_param("~lidar_num_bins", 121)
        self.bin_min_dist = rospy.get_param("~lidar_bin_min_dist", 0.4)

        self.vehicle_width = rospy.get_param("~vehicle_width", 2.0)
        self.lateral_margin = rospy.get_param("~lateral_margin", 0.3)
        self.road_half_width = rospy.get_param("~road_half_width", 3.5)

        self.latest_gap_angle = 0.0
        self.has_blocking_obstacle = False
        self.min_forward_dist = float('inf')

        self._cloud = None
        self.sub = rospy.Subscriber(self.points_topic, PointCloud2, self._cloud_cb, queue_size=1)

    def _cloud_cb(self, msg: PointCloud2):
        self._cloud = msg

    @staticmethod
    def _points_to_numpy(msg: PointCloud2):
        pts = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            pts.append([p[0], p[1], p[2]])
        if not pts: return np.empty((0,3), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    def _to_base_link(self, pts_map, ego_status: EgoVehicleStatus):
        yaw = math.radians(ego_status.heading)
        R = np.array([[ math.cos(-yaw), -math.sin(-yaw)],
                      [ math.sin(-yaw),  math.cos(-yaw)]], dtype=np.float32)
        dx = pts_map[:,0] - ego_status.position.x
        dy = pts_map[:,1] - ego_status.position.y
        xy_rot = np.stack([dx, dy], axis=0)
        xy_bl = R @ xy_rot
        out = np.zeros_like(pts_map)
        out[:,0] = xy_bl[0,:]
        out[:,1] = xy_bl[1,:]
        out[:,2] = pts_map[:,2] - ego_status.position.z
        return out

    def _roi_filter(self, pts_bl):
        x, y, z = pts_bl[:,0], pts_bl[:,1], pts_bl[:,2]
        m = (x >= self.roi_x[0]) & (x <= self.roi_x[1]) & \
            (y >= self.roi_y[0]) & (y <= self.roi_y[1]) & \
            (z >= self.roi_z[0]) & (z <= self.roi_z[1])
        return pts_bl[m]

    def compute_gap_and_ttc(self, ego_status: EgoVehicleStatus):
        if self._cloud is None:
            self.latest_gap_angle = 0.0
            self.has_blocking_obstacle = False
            self.min_forward_dist = float('inf')
            return

        pts = self._points_to_numpy(self._cloud)
        if pts.shape[0] == 0:
            self.latest_gap_angle = 0.0
            self.has_blocking_obstacle = False
            self.min_forward_dist = float('inf')
            return

        if not self.lidar_in_base_link:
            pts = self._to_base_link(pts, ego_status)

        # 간단 지면 필터 + ROI
        pts = pts[pts[:,2] > (self.roi_z[0] + 0.05)]
        pts = self._roi_filter(pts)
        if pts.shape[0] == 0:
            self.latest_gap_angle = 0.0
            self.has_blocking_obstacle = False
            self.min_forward_dist = float('inf')
            return

        # 전방 최소거리(도로 폭 내부)
        within_lane = np.abs(pts[:,1]) <= self.road_half_width
        forward_pts = pts[within_lane & (pts[:,0] > 0.0)]
        if forward_pts.shape[0] > 0:
            self.min_forward_dist = float(np.min(forward_pts[:,0]))
        else:
            self.min_forward_dist = float('inf')

        # 갭 탐색(폴라 히스토그램)
        theta = np.arctan2(pts[:,1], pts[:,0])
        fov_rad = math.radians(self.fov_deg)
        half_fov = fov_rad * 0.5
        valid = (theta >= -half_fov) & (theta <= +half_fov) & (pts[:,0] > 0.0)
        pts = pts[valid]; theta = theta[valid]
        if pts.shape[0] == 0:
            self.latest_gap_angle = 0.0
            self.has_blocking_obstacle = False
            return

        bins = np.linspace(-half_fov, +half_fov, self.num_bins)
        bin_size = bins[1] - bins[0]
        min_dists = np.full(self.num_bins, np.inf, dtype=np.float32)

        idx = np.floor((theta - (-half_fov)) / bin_size).astype(int)
        idx = np.clip(idx, 0, self.num_bins - 1)
        for i, r in zip(idx, pts[:,0]):
            if r < min_dists[i]: min_dists[i] = r

        needed_width = self.vehicle_width + 2.0 * self.lateral_margin
        safe_mask = (min_dists >= self.bin_min_dist)
        r_eval = np.where(np.isfinite(min_dists), min_dists, self.roi_x[1])
        y_at_bin = r_eval * np.sin(bins)
        road_mask = (np.abs(y_at_bin) <= self.road_half_width)
        passable = safe_mask & road_mask

        best_i0, best_i1, best_measure = None, None, -1.0
        i = 0
        while i < self.num_bins:
            if passable[i]:
                j = i
                while j < self.num_bins and passable[j]:
                    j += 1
                segment_bins = np.arange(i, j)
                segment_width_ang = (len(segment_bins)) * bin_size
                seg_r = r_eval[segment_bins]
                seg_r = np.where(np.isfinite(seg_r), seg_r, self.roi_x[1])
                r_mean = float(np.mean(seg_r))
                linear_gap = r_mean * segment_width_ang

                if linear_gap >= needed_width and linear_gap > best_measure:
                    best_measure = linear_gap
                    best_i0, best_i1 = i, j
                i = j
            else:
                i += 1

        if best_i0 is not None:
            i_mid = (best_i0 + best_i1 - 1) * 0.5
            theta_gap = float(-half_fov + i_mid * bin_size)
            self.latest_gap_angle = theta_gap
            self.has_blocking_obstacle = np.isfinite(self.min_forward_dist) and (self.min_forward_dist < max(6.0, needed_width*1.2))
        else:
            self.latest_gap_angle = 0.0
            self.has_blocking_obstacle = True

# ==============================
#         MAIN PURSUIT
# ==============================
class AggressivePursuitNode:
    """
    ego-0(내가 조종) → ego-2(타깃) 추격
    + 라이다 회피 + 코너링 속도 제한
    """
    def __init__(self):
        rospy.init_node("aggressive_pursuit_node", anonymous=True)

        # ====== 토픽 파라미터 ======
        self.pursuer_topic   = rospy.get_param("~pursuer_topic",   "/Ego_topic")        # ego-0 상태
        self.target_topic    = rospy.get_param("~target_topic",    "/Ego-2/Ego_topic")  # ego-2 상태
        self.ctrl_cmd_topic  = rospy.get_param("~ctrl_cmd_topic",  "/ctrl_cmd")         # ego-0 제어 명령
        self.points_topic    = rospy.get_param("~points_topic",    "/points_raw")       # ego-0 LiDAR

        # ====== 맵 경로 ======
        map_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "R_KR_PR_Sangam_NoBuildings")
        self.map_manager = HDMapManager(map_dir)

        # 상태
        self.pursuer_status, self.target_status = None, None

        # Subscribers
        rospy.Subscriber(self.pursuer_topic, EgoVehicleStatus, self.pursuer_status_callback, queue_size=1)
        rospy.Subscriber(self.target_topic,   EgoVehicleStatus, self.target_status_callback, queue_size=1)

        # Publishers
        self.g_path_pub = rospy.Publisher('/global_path', Path, queue_size=1)
        self.l_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.ctrl_pub   = rospy.Publisher(self.ctrl_cmd_topic, CtrlCmd, queue_size=1)

        # LiDAR 모듈
        self.lidar = LiDARObstacleModule(points_topic=self.points_topic)

        # 경로/조향 상태
        self.global_path, self.local_path = None, None
        self.last_steering = 0.0

        # === 주행 파라미터 ===
        self.REVERSE_DRIVING_PENALTY = rospy.get_param("~reverse_penalty", 1.0)
        self.INTERCEPT_TIME_GAIN     = rospy.get_param("~intercept_time_gain", 1.2)
        self.LOOKAHEAD_DISTANCE      = rospy.get_param("~lookahead_distance", 50.0)
        self.GLOBAL_PATH_UPDATE_THRESHOLD = rospy.get_param("~gpath_update_threshold", 15.0)

        # 차량/조향
        self.vehicle_length   = rospy.get_param("~wheelbase", 2.6)
        self.max_steering_rad = rospy.get_param("~max_steering_rad", 0.85)  # ≈ 49°
        self.steer_smoothing_alpha = rospy.get_param("~steer_smoothing_alpha", 0.3)
        self.lfd_gain = rospy.get_param("~lfd_gain", 0.8)
        self.min_lfd, self.max_lfd = rospy.get_param("~min_lfd", 4.0), rospy.get_param("~max_lfd", 30.0)

        # 속도
        self.VELOCITY_KP = rospy.get_param("~vel_kp", 1.5)
        self.MAX_VELOCITY_KPH = rospy.get_param("~v_max_kph", 80.0)

        # 공격적 코너 파라미터
        self.AGGRESSIVE_TURN_DISTANCE = rospy.get_param("~agg_turn_dist", 10.0)
        self.AGGRESSIVE_ANGLE_THRESHOLD = rospy.get_param("~agg_angle_deg", 30.0) * math.pi/180.0
        self.MIN_AGGRESSIVE_VELOCITY_KPH = rospy.get_param("~v_min_agg_kph", 15.0)

        # LiDAR 회피/속도제한
        self.gap_blend_gain_when_blocked = rospy.get_param("~gap_blend_gain_blocked", 0.7)
        self.gap_blend_gain_normal       = rospy.get_param("~gap_blend_gain_normal", 0.3)
        self.lateral_accel_limit = rospy.get_param("~ay_max", 2.5)
        self.v_min_kph = rospy.get_param("~v_min_kph", 5.0)
        self.ttc_thresh = rospy.get_param("~ttc_thresh", 2.0)
        self.ttc_min_dist = rospy.get_param("~ttc_min_dist", 2.0)

        # 종방향 제어 모드 파라미터(★ 추가)
        # 0: throttle, 1: velocity(km/h), 2: accel
        self.longl_mode = rospy.get_param("~longl_mode", 1)
        self.default_throttle = rospy.get_param("~default_throttle", 0.3)

        self.visualizer = PathVisualizer(self.map_manager)

        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop)
        rospy.on_shutdown(self.shutdown_callback)
        rospy.loginfo("ego-0 → ego-2 추격(라이다 회피 + 코너링 속도제한) 노드 시작.")

    def shutdown_callback(self):
        rospy.loginfo("노드 종료. OpenCV 창 닫기.")
        cv2.destroyAllWindows()

    # --------- Callbacks ----------
    def pursuer_status_callback(self, msg): self.pursuer_status = msg
    def target_status_callback(self, msg): self.target_status = msg

    # --------- Main Loop ----------
    def control_loop(self, event):
        if not self.pursuer_status or not self.target_status: return

        # 1) 맵 투영
        ego_proj = self.map_manager.find_projection_on_map(self.pursuer_status.position.x, self.pursuer_status.position.y)
        target_proj = self.map_manager.find_projection_on_map(self.target_status.position.x, self.target_status.position.y)
        if not ego_proj or not target_proj: return

        goal_proj = self._get_intercept_goal_projection(target_proj)

        # 2) 전역 경로 갱신
        if self.global_path is None or self._should_update_global_path(goal_proj):
            rospy.loginfo("전역 경로를 재탐색합니다...")
            path_seq = self.map_manager.get_global_path(ego_proj, goal_proj, self.REVERSE_DRIVING_PENALTY)
            if path_seq:
                self.global_path = self._stitch_links_to_path(path_seq, ego_proj)
                self.g_path_pub.publish(self.global_path)
            else:
                self.global_path = None

        if not self.global_path: return

        # 3) 로컬 경로 추출
        self.local_path = self._extract_local_path()
        if not self.local_path: return
        self.l_path_pub.publish(self.local_path)

        # 4) LiDAR 처리(갭/최소거리/TTC 재료)
        self.lidar.compute_gap_and_ttc(self.pursuer_status)

        # 5) 조향 계산(갭 보정 포함)
        steering_cmd, theta_pp = self.calculate_steering_with_lidar()

        # 6) 속도 계산(코너링/충돌 위험 제한 포함) → m/s
        velocity_cmd_ms = self.calculate_velocity_with_limits(goal_proj, steering_cmd, theta_pp)

        # 7) 제어 퍼블리시 (★ km/h 변환 반영)
        cmd = CtrlCmd()
        cmd.longlCmdType = self.longl_mode
        cmd.steering = float(steering_cmd)

        if self.longl_mode == 1:
            # velocity 모드: 시뮬레이터는 km/h 기대 → 변환해서 전송
            v_cmd_kph = float(velocity_cmd_ms * 3.6)
            cmd.velocity = v_cmd_kph
            # 디버그 로그
            rospy.loginfo(f"[CMD] v_cmd={velocity_cmd_ms:.2f} m/s ({v_cmd_kph:.1f} km/h)")
        elif self.longl_mode == 0:
            # throttle 모드 예시(간단 가감속)
            v_now = max(0.0, self.pursuer_status.velocity.x)
            v_err = float(velocity_cmd_ms - v_now)
            throttle = np.clip(self.default_throttle + 0.05 * v_err, 0.0, 1.0)
            cmd.accel = float(throttle)
            cmd.brake = 0.0
            rospy.loginfo(f"[CMD-throttle] v_now={v_now*3.6:.1f} kph, v_ref={velocity_cmd_ms*3.6:.1f} kph, thr={throttle:.2f}")
        elif self.longl_mode == 2:
            # accel 모드 예시(간단 비례)
            v_now = max(0.0, self.pursuer_status.velocity.x)
            v_err = float(velocity_cmd_ms - v_now)
            cmd.accel = float(np.clip(0.5 * v_err, -2.0, 2.0))  # m/s^2
            rospy.loginfo(f"[CMD-accel] v_now={v_now*3.6:.1f} kph, v_ref={velocity_cmd_ms*3.6:.1f} kph, acc={cmd.accel:.2f}")
        else:
            # 알 수 없는 모드 → velocity로 fallback(km/h)
            v_cmd_kph = float(velocity_cmd_ms * 3.6)
            cmd.longlCmdType = 1
            cmd.velocity = v_cmd_kph
            rospy.logwarn("[CMD] unknown longl_mode, fallback to velocity(km/h)")

        self.ctrl_pub.publish(cmd)

        self.visualizer.update(self.pursuer_status, self.target_status, self.global_path, self.local_path)

    # --------- Helpers ----------
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

    # ---------------- Steering (with LiDAR) ----------------
    def calculate_steering_with_lidar(self):
        theta_pp = 0.0
        if not self.global_path or not self.global_path.poses or not self.local_path or not self.local_path.poses:
            return self.last_steering * 0.9, theta_pp

        current_v = max(0.1, self.pursuer_status.velocity.x)
        lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * current_v))

        found = False
        yaw = math.radians(self.pursuer_status.heading)
        for pose in self.local_path.poses:
            dx = pose.pose.position.x - self.pursuer_status.position.x
            dy = pose.pose.position.y - self.pursuer_status.position.y

            # map → base_link
            lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)

            if lx > 0.1:
                dist = math.sqrt(lx**2 + ly**2)
                if dist >= lfd:
                    theta_pp = math.atan2(ly, lx)
                    found = True
                    break

        if not found:
            smoothed = self.last_steering * 0.9
            return smoothed, theta_pp

        raw_steer = math.atan2(2 * self.vehicle_length * math.sin(theta_pp), lfd)
        raw_steer = max(-self.max_steering_rad, min(self.max_steering_rad, raw_steer))

        gap_ang = self.lidar.latest_gap_angle
        alpha = self.gap_blend_gain_when_blocked if self.lidar.has_blocking_obstacle else self.gap_blend_gain_normal

        steer_from_gap = math.atan2(2 * self.vehicle_length * math.sin(gap_ang), lfd)
        steer_from_gap = max(-self.max_steering_rad, min(self.max_steering_rad, steer_from_gap))

        blended = (1.0 - alpha) * raw_steer + alpha * steer_from_gap

        smoothed = self.last_steering * self.steer_smoothing_alpha + blended * (1 - self.steer_smoothing_alpha)
        smoothed = max(-self.max_steering_rad, min(self.max_steering_rad, smoothed))
        self.last_steering = smoothed
        return smoothed, theta_pp

    # ---------------- Velocity (with corner & TTC limits) ----------------
    def calculate_velocity_with_limits(self, goal_proj, current_steer, theta_pp):
        # 1) 거리 기반 목표속도 (m/s)
        goal_point = self.map_manager.links_data[goal_proj['link_id']]['points'][goal_proj['point_idx']]
        dx = goal_point[0] - self.pursuer_status.position.x
        dy = goal_point[1] - self.pursuer_status.position.y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        target_v = self.VELOCITY_KP * dist_to_goal      # m/s
        v_max = self.MAX_VELOCITY_KPH / 3.6             # m/s 상한

        # 2) 코너링(곡률) 기반 속도 제한 (m/s)
        current_v = max(0.1, self.pursuer_status.velocity.x)
        lfd = max(self.min_lfd, min(self.max_lfd, self.lfd_gain * current_v))
        k_pp = abs(2.0 * math.sin(theta_pp) / max(0.3, lfd))
        if k_pp < 1e-3:
            v_curve_limit = v_max
        else:
            R = 1.0 / k_pp
            v_curve_limit = math.sqrt(max(0.1, self.lateral_accel_limit) * R)

        # 3) (선택) TTC 기반 제한 부분은 기존과 동일/생략 가능
        d_min = self.lidar.min_forward_dist
        if np.isfinite(d_min):
            if d_min <= self.ttc_min_dist:
                v_ttc_limit = 0.0
            else:
                v_ttc_limit = d_min / max(self.ttc_thresh, 0.5)
        else:
            v_ttc_limit = v_max

        # 4) 최종 속도 (m/s)
        v_cmd = min(target_v, v_curve_limit, v_ttc_limit, v_max)

        # 최저속도 바닥 (m/s)
        if dist_to_goal < self.AGGRESSIVE_TURN_DISTANCE:
            min_aggressive_v = self.MIN_AGGRESSIVE_VELOCITY_KPH / 3.6
        else:
            min_aggressive_v = self.v_min_kph / 3.6
        v_cmd = max(min_aggressive_v, v_cmd)

        # 디버그 로그
        rospy.loginfo(f"[VEL] tgt={target_v*3.6:.1f}  curve={v_curve_limit*3.6:.1f}  "
                      f"ttc={v_ttc_limit*3.6:.1f}  vmax={v_max*3.6:.1f} -> cmd={v_cmd*3.6:.1f} kph")
        return float(v_cmd)

    # ---------------- Utilities ----------------
    def convert_points_to_ros_path(self, points):
        ros_path = Path(header=rospy.Header(frame_id='map', stamp=rospy.Time.now()))
        for p in points:
            pose = PoseStamped()
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = float(p[2])
            ros_path.poses.append(pose)
        return ros_path

if __name__ == '__main__':
    try:
        node = AggressivePursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("프로그램 종료")

