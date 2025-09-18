#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, cv2, math, numpy as np
from sensor_msgs.msg import CompressedImage
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class LaneKeepFrontViewMinimal:
    def __init__(self):
        rospy.init_node("lane_keep_frontview_min_overlay_only", anonymous=False)

        # -------- Params --------
        self.target_speed_kmh = rospy.get_param("~target_speed_kmh", 25.0)
        self.k_stanley        = rospy.get_param("~k_stanley", 0.6)
        self.max_steer_rad    = rospy.get_param("~max_steer_rad", 0.6)
        self.lane_width_m     = rospy.get_param("~lane_width_m", 3.5)
        self.roi_ratio_top    = rospy.get_param("~roi_ratio_top", 0.5)   # 하단 50%
        self.hough_threshold  = rospy.get_param("~hough_threshold", 60)
        self.hough_min_len    = rospy.get_param("~hough_min_len", 60)
        self.hough_max_gap    = rospy.get_param("~hough_max_gap", 20)
        self.use_yellow       = rospy.get_param("~use_yellow", True)
        self.ema_alpha        = rospy.get_param("~ema_alpha", 0.5)
        self.use_ego_heading  = rospy.get_param("~use_ego_heading", False)  # 기본 False(카메라 기준)
        self.jpg_quality      = rospy.get_param("~jpg_quality", 70)          # 낮춰서 리소스 절약

        # -------- State --------
        self.v_mps, self.yaw_deg = 0.0, 0.0
        self.pix2m_x = None
        self.ey_ema, self.epsi_ema = 0.0, 0.0
        self.t0 = rospy.get_time()
        self.int_err = 0.0
        self.speed_kp, self.speed_ki = 0.25, 0.05

        # -------- Pub/Sub --------
        self.pub_ctrl = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.pub_vis  = rospy.Publisher("/lane_vis/compressed", CompressedImage, queue_size=1)

        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_cb,
                         queue_size=1, buff_size=2**22)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_cb, queue_size=1)

        rospy.loginfo("[lane_keep_frontview_min_overlay_only] started")
        rospy.spin()

    # ================= Vehicle state =================
    def status_cb(self, m:EgoVehicleStatus):
        vx, vy = float(m.velocity.x), float(m.velocity.y)
        self.v_mps = math.hypot(vx, vy)

        if not self.use_ego_heading:
            return
        hd = float(m.heading)
        self.yaw_deg = hd if abs(hd) > 6.283185307 else math.degrees(hd)
        while self.yaw_deg > 180: self.yaw_deg -= 360
        while self.yaw_deg < -180: self.yaw_deg += 360

    # ================= Perception + Control =================
    def image_cb(self, msg:CompressedImage):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None: return
        H, W = frame.shape[:2]

        # ROI(하단)
        y0 = int(H * (1.0 - self.roi_ratio_top))
        roi = frame[y0:, :]
        rh, rw = roi.shape[:2]

        # --- 1) 실제 차선 픽셀 마스크(= "실제 차선" 표시용) ---
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white  = cv2.inRange(hsv, (0, 0, 200), (180, 60, 255))
        yellow = cv2.inRange(hsv, (18, 60, 150), (40, 255, 255)) if self.use_yellow else np.zeros_like(white)
        color_mask = cv2.bitwise_or(white, yellow)

        edges = cv2.Canny(cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY),(5,5),0), 70, 150)
        binary = cv2.bitwise_or(color_mask, edges)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8), 1)

        # --- 2) Hough로 좌/우 차선 모델(y = m x + b) ---
        lines = cv2.HoughLinesP(binary, 1, np.pi/180, threshold=self.hough_threshold,
                                minLineLength=self.hough_min_len, maxLineGap=self.hough_max_gap)
        L, R = None, None
        if lines is not None:
            left_lines, right_lines = [], []
            for (x1,y1,x2,y2) in lines[:,0,:]:
                if min(y1,y2) < rh*0.15:  # ROI 상단 노이즈 제거
                    continue
                dx = x2-x1
                if dx == 0: continue
                slope = (y2-y1)/float(dx)  # 이미지: x→우+, y→아래+
                if slope < -0.3: left_lines.append((x1,y1,x2,y2))
                elif slope > 0.3: right_lines.append((x1,y1,x2,y2))
            L = self.fit_weighted_line(left_lines)
            R = self.fit_weighted_line(right_lines)

        # 픽셀-미터 스케일(양쪽 있을 때)
        lane_pix_width = None
        if L and R:
            yb = rh-1
            xl = self.x_at_y(L, yb); xr = self.x_at_y(R, yb)
            lane_pix_width = max(1.0, xr - xl)
            self.pix2m_x = self.lane_width_m / lane_pix_width
        elif L and not R and self.pix2m_x:
            yb = rh-1; xl = self.x_at_y(L, yb); xr = xl + self.lane_width_m/self.pix2m_x
            R = self.line_from_two_points((int(xr), yb), (int(xr-50), max(yb-50,0)))
        elif R and not L and self.pix2m_x:
            yb = rh-1; xr = self.x_at_y(R, yb); xl = xr - self.lane_width_m/self.pix2m_x
            L = self.line_from_two_points((int(xl), yb), (int(xl+50), max(yb-50,0)))

        # --- 3) 중심선·오프셋·헤딩(카메라 기준 전방=이미지 아래) ---
        ey_m, epsi = 0.0, 0.0
        centers = []
        if L or R:
            y_samples = np.arange(int(rh*0.55), rh, 5).astype(int)
            for yy in y_samples:
                xs = []
                if L: xs.append(self.x_at_y(L, yy))
                if R: xs.append(self.x_at_y(R, yy))
                if len(xs)==2:
                    cx = 0.5*(xs[0]+xs[1])
                elif len(xs)==1 and lane_pix_width:
                    cx = xs[0] + ( lane_pix_width*0.5 if L else -lane_pix_width*0.5 )
                else:
                    continue
                centers.append((cx, yy))

            if len(centers) >= 2:
                cx2, cy2 = centers[-1]
                i1 = max(0, len(centers)-1 - max(3, int(0.2*len(centers))))
                cx1, cy1 = centers[i1]

                pix2m = self.pix2m_x if self.pix2m_x else 0.02
                ey_m  = (cx2 - rw*0.5) * pix2m

                heading_path = math.atan2((cy2-cy1), (cx2-cx1+1e-6))
                veh_heading  = math.pi/2 if not self.use_ego_heading else math.radians(self.yaw_deg)
                epsi = self.wrap_rad(heading_path - veh_heading)

        # 스무딩
        self.ey_ema   = self.ema_alpha*ey_m + (1-self.ema_alpha)*self.ey_ema
        self.epsi_ema = self.ema_alpha*epsi  + (1-self.ema_alpha)*self.epsi_ema

        # --- 4) 제어 (초기 조향/가속 제한) ---
        v = max(self.v_mps, 0.1)
        delta = self.epsi_ema + math.atan2(self.k_stanley*self.ey_ema, v)
        elapsed = rospy.get_time() - self.t0
        steer_limit = 0.25 if elapsed < 1.0 else self.max_steer_rad
        delta = float(np.clip(delta, -steer_limit, steer_limit))
        v_ref = 0.0 if elapsed < 0.5 else (self.target_speed_kmh/3.6)
        verr  = v_ref - v
        self.int_err = np.clip(self.int_err + verr*0.02, -5.0, 5.0)
        accel_cmd = float(np.clip(self.speed_kp*verr + self.speed_ki*self.int_err, -1.0, 1.0))
        accel, brake = (accel_cmd, 0.0) if accel_cmd>=0 else (0.0, -accel_cmd)

        cmd = CtrlCmd(); cmd.longlCmdType=0; cmd.steering=delta; cmd.accel=accel; cmd.brake=brake
        self.pub_ctrl.publish(cmd)

        # --- 5) 단일 디버그 시각화(검은 배경에 실제차선+검출 오버레이) ---
        canvas = np.zeros((rh, rw, 3), dtype=np.uint8)

        # 실제 차선(마스크)을 회색으로 페인트
        mask_vis = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        mask_vis = (mask_vis * 0.6).astype(np.uint8)  # 어둡게
        canvas = cv2.max(canvas, mask_vis)            # 회색 픽셀로 남김

        # 좌/우 차선 직선 모델
        def draw_line(model, color):
            if not model: return
            y1, y2 = int(rh*0.2), rh-2
            x1 = self.x_at_y(model, y1); x2 = self.x_at_y(model, y2)
            if x1 is not None and x2 is not None:
                cv2.line(canvas, (int(x1), y1), (int(x2), y2), color, 3)

        draw_line(L, (255,0,0))     # 왼쪽: 파랑
        draw_line(R, (0,0,255))     # 오른쪽: 빨강

        # 중심선(초록 점)
        for (cx, cy) in centers:
            cv2.circle(canvas, (int(cx), int(cy)), 3, (0,255,0), -1)

        ok, buf = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpg_quality)])
        if ok:
            m = CompressedImage(); m.header.stamp=rospy.Time.now(); m.format="jpeg"; m.data=np.array(buf).tobytes()
            self.pub_vis.publish(m)

    # ================= helpers =================
    @staticmethod
    def fit_weighted_line(lines):
        if not lines: return None
        xs, ys, ws = [], [], []
        for x1,y1,x2,y2 in lines:
            w = math.hypot(x2-x1, y2-y1) + 1e-6
            xs += [x1, x2]; ys += [y1, y2]; ws += [w, w]
        xs, ys, ws = np.array(xs), np.array(ys), np.array(ws)
        X = np.vstack([xs, np.ones_like(xs)]).T
        W = np.diag(ws)
        try:
            m, b = np.linalg.inv(X.T @ W @ X) @ (X.T @ W @ ys)
            return ("yx", m, b)  # y = m x + b
        except np.linalg.LinAlgError:
            return None

    @staticmethod
    def x_at_y(model, y):
        if model is None: return None
        _, m, b = model
        if abs(m) < 1e-6: return None
        return (y - b) / m

    @staticmethod
    def line_from_two_points(p1, p2):
        x1,y1 = p1; x2,y2 = p2
        if x2==x1: return None
        m = (y2-y1)/float(x2-x1); b = y1 - m*x1
        return ("yx", m, b)

    @staticmethod
    def wrap_rad(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

if __name__ == "__main__":
    LaneKeepFrontViewMinimal()

