#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, math, time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from std_msgs.msg import Header
from scipy.spatial import KDTree
from collections import deque, defaultdict

class HDMapManager:
    def __init__(self, map_dir):
        link_file = os.path.join(map_dir, "R_KR_PR_Sangam_NoBuildings", "link_set.json")
        node_file = os.path.join(map_dir, "R_KR_PR_Sangam_NoBuildings", "node_set.json")
        rospy.loginfo(f"맵 데이터 로딩: {link_file}, {node_file}")

        with open(link_file, "r", encoding="utf-8") as f:
            data = json.load(f)
        # ---- Robust link_set formats ----
        # Supported structures:
        # 1) {"link_set":{"links":[...]}}, 2) {"links":[...]}, 3) [...]
        if isinstance(data, dict):
            if "link_set" in data and isinstance(data["link_set"], dict) and "links" in data["link_set"]:
                self.links = data["link_set"]["links"]
            elif "links" in data and isinstance(data["links"], list):
                self.links = data["links"]
            else:
                # Fallback: try to find a list value that looks like links
                cand = None
                for k,v in data.items():
                    if isinstance(v, list) and v and isinstance(v[0], dict) and ("points" in v[0] or "point" in v[0]):
                        cand = v; break
                if cand is None:
                    raise ValueError("link_set.json 형식을 해석할 수 없습니다. 최상위에 links 배열이 필요합니다.")
                self.links = cand
        elif isinstance(data, list):
            self.links = data
        else:
            raise ValueError("link_set.json 최상위 타입이 dict/list가 아닙니다.")


        with open(node_file, "r", encoding="utf-8") as f:
            self.nodes = json.load(f)

        # Flatten points for KDTree
        self.link_points = []      # [ [x,y], ... ]
        self.link_of_point = []    # index -> (link_id, point_idx)
        self.min_x = self.min_y = float("inf")
        self.max_x = self.max_y = -float("inf")
        for link in self.links:
            pts = link.get("points") or link.get("point") or (link.get("lane") or {}).get("points")
            for i,p in enumerate(pts):
                x, y = p[0], p[1]
                self.min_x, self.min_y = min(self.min_x, x), min(self.min_y, y)
                self.max_x, self.max_y = max(self.max_x, x), max(self.max_y, y)
                self.link_points.append([x,y])
                self.link_of_point.append((link["idx"], i))
        self.kdt = KDTree(np.array(self.link_points, dtype=np.float64))
        rospy.loginfo("맵 데이터 로딩 및 KDTree 생성 완료!")
        rospy.loginfo(f"링크 수: {len(self.links)}, 기하학 인접 그래프 노드 수: {len(getattr(self, 'adj_links', {}))}")

        # Graph (link adjacency) – build naive adjacency using start/end node ids if present
        self.adj = defaultdict(list)
        for lk in self.links:
            s = lk.get("from_node", lk.get("start_node"))
            e = lk.get("to_node", lk.get("end_node"))
            if s is not None and e is not None:
                self.adj[s].append((e, lk["idx"]))
        # Index by link_id -> (from_node, to_node)
        self.link2nodes = {}
        for lk in self.links:
            s = lk.get("from_node", lk.get("start_node"))
            e = lk.get("to_node", lk.get("end_node"))
            self.link2nodes[lk["idx"]] = (s, e)

        # Build node->link reverse to traverse link graph
        self.node2links_from = defaultdict(list)
        for lk in self.links:
            s = self.link2nodes[lk["idx"]][0]
            self.node2links_from[s].append(lk["idx"])
        # --- Build geometric adjacency (fallback when node ids are missing or sparse) ---
        # For each link, connect to any next link whose START point is within EPS of this link's END point.
        self.link_end_xy = {}
        self.link_start_xy = {}
        for lk in self.links:
            arr = lk.get("points") or lk.get("point") or (lk.get("lane") or {}).get("points")
            if not arr or len(arr) < 2:
                continue
            self.link_start_xy[lk["idx"]] = (float(arr[0][0]), float(arr[0][1]))
            self.link_end_xy[lk["idx"]]   = (float(arr[-1][0]), float(arr[-1][1]))

        # KDTree over START points to find candidates near each END point
        import numpy as _np
        from scipy.spatial import KDTree as _KD
        start_ids = []
        start_xy  = []
        for lid, xy in self.link_start_xy.items():
            start_ids.append(lid); start_xy.append(xy)
        if start_xy:
            kd = _KD(_np.array(start_xy, dtype=_np.float64))
            EPS = 2.0  # meters, adjust if needed
            self.adj_links = {lid: [] for lid in self.link_start_xy.keys()}
            for lid, end_xy in self.link_end_xy.items():
                idxs = kd.query_ball_point(_np.array(end_xy, dtype=_np.float64), EPS)
                for i in idxs:
                    nxt = start_ids[i]
                    if nxt != lid:
                        self.adj_links.setdefault(lid, []).append(nxt)
        else:
            self.adj_links = {}


    def find_projection_on_map(self, x, y):
        if not self.link_points:
            return None
        d, idx = self.kdt.query([x,y], k=1)
        link_id, point_idx = self.link_of_point[idx]
        return {"link_id": link_id, "point_idx": point_idx, "dist": float(d)}

    
    def find_shortest_link_sequence(self, start_link, goal_link, max_hops=400):
        """BFS over links. Prefer node graph; fall back to geometric adjacency when node graph is missing."""
        if start_link == goal_link:
            return [start_link]
        use_node_graph = bool(self.node2links_from) and all(k is not None for k in self.link2nodes.get(start_link, (None,None))) and all(k is not None for k in self.link2nodes.get(goal_link, (None,None)))
        if use_node_graph:
            # Node-based traversal
            start_node = self.link2nodes[start_link][1]  # end of start_link
            q, visited, parent = deque(), set(), {}
            for lk in self.node2links_from.get(start_node, []):
                q.append(lk); visited.add(lk); parent[lk] = None
            while q and len(visited) <= max_hops:
                cur = q.popleft()
                if cur == goal_link:
                    path = [cur]
                    while parent[cur] is not None:
                        cur = parent[cur]
                        path.append(cur)
                    path.reverse()
                    return [start_link] + path
                cur_end = self.link2nodes.get(cur, (None, None))[1]
                for nxt in self.node2links_from.get(cur_end, []):
                    if nxt not in visited:
                        visited.add(nxt); parent[nxt] = cur; q.append(nxt)
            return []
        # Geometric fallback
        if not hasattr(self, 'adj_links') or not self.adj_links:
            return []
        q, visited, parent = deque(), set(), {}
        for nxt in self.adj_links.get(start_link, []):
            q.append(nxt); visited.add(nxt); parent[nxt] = None
        while q and len(visited) <= max_hops:
            cur = q.popleft()
            if cur == goal_link:
                path = [cur]
                while parent[cur] is not None:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return [start_link] + path
            for nxt in self.adj_links.get(cur, []):
                if nxt not in visited:
                    visited.add(nxt); parent[nxt] = cur; q.append(nxt)
        return []

        start_node = self.link2nodes[start_link][1]  # end of start_link
        goal_node_start, goal_node_end = self.link2nodes[goal_link]

        q = deque()
        visited = set()
        parent = {}
        # enqueue all links leaving start_node
        for lk in self.node2links_from[start_node]:
            q.append(lk); visited.add(lk); parent[lk] = None

        while q and len(visited) <= max_hops:
            cur = q.popleft()
            if cur == goal_link:
                # reconstruct
                path = [cur]
                while parent[cur] is not None:
                    cur = parent[cur]
                    path.append(cur)
                path.reverse()
                return [start_link] + path  # include start_link at head
            cur_end = self.link2nodes[cur][1]
            for nxt in self.node2links_from[cur_end]:
                if nxt not in visited:
                    visited.add(nxt); parent[nxt] = cur; q.append(nxt)
        return []

    def ros_path_from_links(self, link_seq, start_proj):
        pts = []
        for lid in link_seq:
            link = next((lk for lk in self.links if lk["idx"] == lid), None)
            if not link: 
                continue
            arr = link.get("points") or link.get("point") or (link.get("lane") or {}).get("points")
            for p in arr:
                pts.append((p[0], p[1], (p[2] if len(p)>=3 else 0.0)))
        if not pts:
            return None
        path = Path(header=Header(frame_id="map", stamp=rospy.Time.now()))
        for (x,y,z) in pts:
            ps = PoseStamped()
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(z)
            path.poses.append(ps)
        return path

class PathVisualizer:
    WIN_NAME = "Hierarchical Path Planning"
    VIEW_WIDTH = 600
    VIEW_HEIGHT = 600
    SCALE = 1.5  # pixels per map unit

    def __init__(self, map_manager: HDMapManager):
        self.map_manager = map_manager
        w = int((map_manager.max_x - map_manager.min_x) * self.SCALE) + 200
        h = int((map_manager.max_y - map_manager.min_y) * self.SCALE) + 200
        self.map_background = np.zeros((h, w, 3), dtype=np.uint8)
        self._draw_all_links(self.map_background)
        rospy.loginfo("배경 맵 이미지 생성 완료!")

    def _draw_all_links(self, img):
        for lk in self.map_manager.links:
            arr = lk.get("points") or lk.get("point") or (lk.get("lane") or {}).get("points")
            for i in range(len(arr)-1):
                x1,y1 = arr[i][0], arr[i][1]
                x2,y2 = arr[i+1][0], arr[i+1][1]
                p1 = self._world_to_pixel(x1,y1)
                p2 = self._world_to_pixel(x2,y2)
                cv2.line(img, p1, p2, (50,50,50), 1)

    def _world_to_pixel(self, x, y):
        px = int((x - self.map_manager.min_x) * self.SCALE) + 100
        py = self.map_background.shape[0] - (int((y - self.map_manager.min_y) * self.SCALE) + 100)
        return (px, py)

    def draw(self, ego_status, global_path, local_path):
        if ego_status is None:
            return
        cx = int((ego_status.position.x - self.map_manager.min_x) * self.SCALE) + 100
        cy = self.map_background.shape[0] - (int((ego_status.position.y - self.map_manager.min_y) * self.SCALE) + 100)
        half_w, half_h = self.VIEW_WIDTH//2, self.VIEW_HEIGHT//2
        cx = max(half_w, min(self.map_background.shape[1]-half_w-1, cx))
        cy = max(half_h, min(self.map_background.shape[0]-half_h-1, cy))
        view = cv2.getRectSubPix(self.map_background, (self.VIEW_WIDTH, self.VIEW_HEIGHT), (float(cx), float(cy)))
        # draw ego + heading arrow
        cv2.circle(view, (half_w, half_h), 4, (255,255,255), -1)
        yaw = math.radians(ego_status.heading)
        tip = (int(half_w + 20*math.cos(yaw)), int(half_h - 20*math.sin(yaw)))
        cv2.arrowedLine(view, (half_w, half_h), tip, (255,255,255), 2, tipLength=0.3)
        # draw global path (yellow)
        if global_path:
            for i in range(1, len(global_path.poses)):
                p0 = global_path.poses[i-1].pose.position
                p1 = global_path.poses[i].pose.position
                a = self._world_to_pixel(p0.x, p0.y); b = self._world_to_pixel(p1.x, p1.y)
                a=(a[0]-cx+half_w, a[1]-cy+half_h); b=(b[0]-cx+half_w, b[1]-cy+half_h)
                cv2.line(view, a, b, (0,255,255), 2)
        # draw local path (green)
        if local_path:
            for i in range(1, len(local_path.poses)):
                p0 = local_path.poses[i-1].pose.position
                p1 = local_path.poses[i].pose.position
                a = self._world_to_pixel(p0.x, p0.y); b = self._world_to_pixel(p1.x, p1.y)
                a=(a[0]-cx+half_w, a[1]-cy+half_h); b=(b[0]-cx+half_w, b[1]-cy+half_h)
                cv2.line(view, a, b, (0,255,0), 2)
        # HUD
        glen = len(global_path.poses) if global_path else 0
        llen = len(local_path.poses) if local_path else 0
        cv2.putText(view, f"glen={glen} llen={llen}", (10,20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
        cv2.imshow(self.WIN_NAME, view); cv2.waitKey(1)

class PursuitNode:
    WHEELBASE = 2.7  # meters
    MAX_STEER_RAD = 0.6  # ~34 deg

    LOOKAHEAD = 40.0  # meters
    def __init__(self):
        rospy.init_node("aggressive_pursuit_node", anonymous=True)
        map_dir = os.path.dirname(os.path.abspath(__file__))
        self.map_manager = HDMapManager(map_dir)
        self.visualizer = PathVisualizer(self.map_manager)

        self.pursuer_status = None
        self.target_status = None

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self._cb_pursuer, queue_size=1)
        rospy.Subscriber("/Ego-2/Ego_topic", EgoVehicleStatus, self._cb_target, queue_size=1)

        self.g_path_pub = rospy.Publisher("/global_path", Path, queue_size=1)
        self.l_path_pub = rospy.Publisher("/local_path", Path, queue_size=1)
        self.ctrl_pub  = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)  # 환경에 따라 /Ego/ctrl_cmd

        self.global_path = None
        self.local_path  = None
        self._last_goal_proj = None
        self._last_start_proj = None
        self._last_plan_time = 0.0
        self._last_ego_xy = None

        rospy.Timer(rospy.Duration(1.0/20.0), self.control_loop)
        rospy.on_shutdown(self._on_shutdown)
        rospy.loginfo("Ego가 Ego-2를 추격하는 노드 시작")

    def _on_shutdown(self):
        cv2.destroyAllWindows()
        rospy.loginfo("프로그램 종료")

    def _cb_pursuer(self, msg):
        self.pursuer_status = msg
        rospy.loginfo_throttle(1.0, f"[pursuer] x={msg.position.x:.1f}, y={msg.position.y:.1f}")

    def _cb_target(self, msg):
        self.target_status = msg
        rospy.loginfo_throttle(1.0, f"[target ] x={msg.position.x:.1f}, y={msg.position.y:.1f}")

    # ---- Planner helpers ----
    def _should_update_global_path(self, goal_proj):
        """Decide when to rebuild the global path.
        Triggers:
          - first run or empty path
          - goal link changed OR goal moved far along the same link
          - ego progressed far along the current path from the last start projection
        """
        if not self.global_path or not self.global_path.poses:
            self._last_goal_proj = goal_proj
            self._last_start_proj = self.map_manager.find_projection_on_map(
                self.pursuer_status.position.x, self.pursuer_status.position.y
            )
            return True

        end_pos = self.global_path.poses[-1].pose.position
        end_proj = self.map_manager.find_projection_on_map(end_pos.x, end_pos.y)
        if not end_proj:
            self._last_goal_proj = goal_proj
            return True
        if goal_proj["link_id"] != end_proj["link_id"]:
            self._last_goal_proj = goal_proj
            return True
        if self._last_goal_proj and goal_proj["link_id"] == self._last_goal_proj["link_id"]:
            if abs(goal_proj["point_idx"] - self._last_goal_proj.get("point_idx", 0)) >= 5:
                self._last_goal_proj = goal_proj
                return True

        cur_start = self.map_manager.find_projection_on_map(
            self.pursuer_status.position.x, self.pursuer_status.position.y
        )
        if self._last_start_proj:
            moved_far = False
            try:
                if cur_start["link_id"] != self._last_start_proj["link_id"]:
                    moved_far = True
                else:
                    moved_far = abs(cur_start["point_idx"] - self._last_start_proj.get("point_idx", 0)) >= 5
            except Exception:
                moved_far = True
            if moved_far:
                self._last_start_proj = cur_start
                self._last_goal_proj = goal_proj
                return True
        else:
            self._last_start_proj = cur_start
        self._last_goal_proj = goal_proj
        return False

    def _rebuild_global_path(self, goal_proj):
        ego_proj = self.map_manager.find_projection_on_map(self.pursuer_status.position.x, self.pursuer_status.position.y)
        if ego_proj is None:
            rospy.logwarn_throttle(1.0, "[planner] ego projection failed")
            return
        seq = self.map_manager.find_shortest_link_sequence(ego_proj["link_id"], goal_proj["link_id"])
        if not seq:
            rospy.logwarn_throttle(1.0, "[planner] no link sequence found")
            return
        self.global_path = self.map_manager.ros_path_from_links(seq, ego_proj)
        self.g_path_pub.publish(self.global_path)
        rospy.loginfo_throttle(0.5, f"[planner] REPLAN: len={len(self.global_path.poses)}")

    def _heading_of_points(self, p0, p1):
        dx, dy = (p1[0]-p0[0], p1[1]-p0[1])
        return math.atan2(dy, dx) if (dx*dx+dy*dy)>1e-6 else 0.0

    def _lateral_error_to_segment(self, x, y, p0, p1):
        vx, vy = (p1[0]-p0[0], p1[1]-p0[1])
        wx, wy = (x - p0[0], y - p0[1])
        seg_len2 = vx*vx + vy*vy
        if seg_len2 < 1e-6:
            return math.hypot(wx, wy)
        t = max(0.0, min(1.0, (wx*vx + wy*vy)/seg_len2))
        projx, projy = (p0[0] + t*vx, p0[1] + t*vy)
        return ((x - projx)*(-vy) + (y - projy)*(vx)) / math.sqrt(seg_len2)

    # ---- Control loop ----
    def control_loop(self, event):
        if self.pursuer_status is None or self.target_status is None:
            return
        now = time.time()
        ego_xy = (self.pursuer_status.position.x, self.pursuer_status.position.y)
        moved_far = False
        if self._last_ego_xy is not None:
            dx = ego_xy[0]-self._last_ego_xy[0]; dy = ego_xy[1]-self._last_ego_xy[1]
            moved_far = (dx*dx+dy*dy) >= 1.0
        self._last_ego_xy = ego_xy

        goal_proj = self.map_manager.find_projection_on_map(self.target_status.position.x, self.target_status.position.y)
        if goal_proj is None:
            rospy.logwarn_throttle(1.0, "[planner] goal projection failed")
            return

        need_update = self._should_update_global_path(goal_proj)
        if (now - self._last_plan_time) > 1.0 or moved_far:
            need_update = True
        if need_update:
            self._rebuild_global_path(goal_proj)
            self._last_plan_time = now

        # Local path: simple forward slice along global path
        self.local_path = self._make_local_path(ego_xy, self.LOOKAHEAD)
        if self.local_path:
            self.l_path_pub.publish(self.local_path)

        # Safety-gated throttle + Pure Pursuit steering
        cmd = CtrlCmd(); cmd.longlCmdType = 0
        steer_cmd = 0.0
        if self.local_path and len(self.local_path.poses) >= 2:
            tgt = self.local_path.poses[-1].pose.position
            ex, ey = ego_xy
            yaw = math.radians(self.pursuer_status.heading)
            dx = tgt.x - ex; dy = tgt.y - ey
            x_r =  math.cos(-yaw)*dx - math.sin(-yaw)*dy
            y_r =  math.sin(-yaw)*dx + math.cos(-yaw)*dy
            Ld = max(1.0, math.hypot(x_r, y_r))
            alpha = math.atan2(y_r, x_r)
            steer_cmd = math.atan2(2.0 * self.WHEELBASE * math.sin(alpha), Ld)
            steer_cmd = max(-self.MAX_STEER_RAD, min(self.MAX_STEER_RAD, steer_cmd))
        can_accel = False
        if self.global_path and self.global_path.poses:
            poses = self.local_path.poses if self.local_path and len(self.local_path.poses) >= 2 else self.global_path.poses
            p0 = (poses[0].pose.position.x, poses[0].pose.position.y)
            p1 = (poses[min(1, len(poses)-1)].pose.position.x, poses[min(1, len(poses)-1)].pose.position.y)
            path_yaw = self._heading_of_points(p0, p1)
            ego_yaw = math.radians(self.pursuer_status.heading)  # MORAI heading in deg
            heading_err = abs((math.degrees(path_yaw - ego_yaw) + 180) % 360 - 180)
            cte = self._lateral_error_to_segment(ego_xy[0], ego_xy[1], p0, p1)
            if heading_err < 15.0 and abs(cte) < 0.8:
                can_accel = True
            rospy.loginfo_throttle(0.5, f"[ctrl] head_err={heading_err:.1f}deg cte={cte:.2f}m accel={'on' if can_accel else 'off'}")
        cmd.accel, cmd.brake = (0.2, 0.0) if can_accel else (0.0, 0.2)
        cmd.steering = float(steer_cmd)
        self.ctrl_pub.publish(cmd)

        # Visual
        try:
            self.visualizer.draw(self.pursuer_status, self.global_path, self.local_path)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"[viz] draw error: {e}")

    
def _nearest_index(self, pts, ego_xy):
    d = np.sum((pts - np.array(ego_xy))**2, axis=1)
    return int(np.argmin(d))

def _make_local_path(self, ego_xy, lookahead):
    if not self.global_path or not self.global_path.poses:
        return None
    pts = np.array([[p.pose.position.x, p.pose.position.y] for p in self.global_path.poses], dtype=np.float64)
    start_idx = self._nearest_index(pts, ego_xy)
    acc = 0.0
    end_idx = start_idx
    for i in range(start_idx, len(pts)-1):
        seg = np.linalg.norm(pts[i+1] - pts[i])
        acc += seg
        end_idx = i+1
        if acc >= lookahead:
            break
    loc = Path(header=Header(frame_id="map", stamp=rospy.Time.now()))
    for i in range(start_idx, end_idx+1):
        loc.poses.append(self.global_path.poses[i])
    return loc


if __name__ == "__main__":
    try:
        node = PursuitNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
