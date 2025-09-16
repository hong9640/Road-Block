#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
import math
from collections import deque
np.set_printoptions(suppress=True)

class LaneKeeperFrontView:
    def __init__(self):
        rospy.init_node("lane_keeper_frontview", anonymous=False)
        rospy.loginfo("[lane_keeper_frontview] start")

        # ====== 파라미터 ======
        self.target_speed_kmh   = rospy.get_param("~target_speed_kmh", 30.0)   # 목표 속도
        self.max_steer_rad      = rospy.get_param("~max_steer_rad", 0.6)       # 스티어 제한(라디안)
        self.k_stanley          = rospy.get_param("~k_stanley", 0.6)           # Stanley 게인
        self.look_y_pixels      = rospy.get_param("~look_y_pixels", 60)        # 하단에서 lookahead용 y
        self.lane_width_m       = rospy.get_param("~lane_width_m", 3.5)        # 차로 폭(보정용)
        self.use_yellow         = rospy.get_param("~use_yellow", True)         # 중앙 황색 차선 사용
        self.min_lane_height_px = rospy.get_param("~min_lane_height_px", 120)  # 유효 차선 세로 길이 하한
        self.ema_alpha          = rospy.get_param("~ema_alpha", 0.5)           # 중심선/오차 EMA
        self.speed_kp           = rospy.get_param("~speed_kp", 0.25)           # 속도 PI
        self.speed_ki           = rospy.get_param("~speed_ki", 0.05)
        self.debug_viz          = rospy.get_param("~debug_viz", False)         # OpenCV 창 보기(로컬만)

        # 상태
        self.last_center_x      = None
        self.last_heading_deg   = None
        self.pix2m_x            = None  # 프레임 하단에서의 m/px
        self.ey_ema             = 0.0
        self.epsi_ema           = 0.0
        self.int_err            = 0.0
        self.v_mps              = 0.0
        self.yaw_deg            = 0.0

        # 버퍼
        self.centerline_hist = deque(maxlen=5)  # 안정화용

        # ROS I/O
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1)
        self.path_pub = rospy.Publisher("/lane_path", Path, queue_size=1)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.image_cb, queue_size=1, buff_size=2**22)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.status_cb, queue_size=1)

        rospy.loginfo("[lane_keeper_frontview] ready")
        rospy.spin()

    # ====== 차량 상태 수신 ======
    def status_cb(self, msg: EgoVehicleStatus):
        # 속도는 x/y/z로 올 수 있음: m/s
        vx, vy, vz = msg.velocity.x, msg.velocity.y, msg.velocity.z
        self.v_mps = float(np.hypot(vx, vy))
        # heading: 라디안/도 중 모라이는 일반적으로 라디안(환경에 따라 다를 수 있음)
        # 여기서는 라디안 가정 -> 도로 변환
        self.yaw_deg = math.degrees(msg.heading) if abs(msg.heading) < 10.0 else msg.heading

    # ====== 메인 콜백 ======
    def image_cb(self, msg: CompressedImage):
        # 1) 이미지 복원
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]

        # 2) ROI: 하단 60%
        y0 = int(h * 0.4)
        roi = frame[y0:, :]
        rh, rw = roi.shape[:2]

        # 3) 전처리: 색상 + 에지
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 흰색 차선 마스크
        white = cv2.inRange(hsv, (0, 0, 200), (180, 60, 255))

        # 황색(중앙선) 마스크(선택)
        yellow = cv2.inRange(hsv, (15, 80, 120), (35, 255, 255)) if self.use_yellow else np.zeros_like(white)

        color_mask = cv2.bitwise_or(white, yellow)

        # 에지 보조
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 80, 160)

        # 통합 바이너리
        binary = cv2.bitwise_or(color_mask, edges)

        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)

        # 4) 좌/우 차선 추출(컨투어 기반)
        left_pts, right_pts = self.extract_lane_points(binary)

        # 검출 실패 처리
        if len(left_pts) < self.min_lane_height_px // 2 and len(right_pts) < self.min_lane_height_px // 2:
            self.publish_safe_stop("[warn] lane lost (both)")
            return

        # 5) 2차 다항식 피팅 (x = a*y^2 + b*y + c)  ; y는 ROI 좌표(0~rh-1, 아래로 증가)
        left_fit, right_fit = None, None
        if len(left_pts) >= 30:
            left_fit = np.polyfit(left_pts[:,1], left_pts[:,0], 2)  # x=f(y)
        if len(right_pts) >= 30:
            right_fit = np.polyfit(right_pts[:,1], right_pts[:,0], 2)

        # 6) 중심선 구성
        ys = np.arange(0, rh, 5, dtype=np.float32)
        left_xs  = self.eval_poly(left_fit, ys)  if left_fit  is not None else None
        right_xs = self.eval_poly(right_fit, ys) if right_fit is not None else None

        # 한쪽만 보일 때 보정: 차선 폭 사용
        if left_xs is None and right_xs is not None:
            left_xs = right_xs - (self.lane_width_pixels(right_xs, rh) or 180)
        if right_xs is None and left_xs is not None:
            right_xs = left_xs + (self.lane_width_pixels(left_xs, rh) or 180)

        if left_xs is None or right_xs is None:
            self.publish_safe_stop("[warn] lane lost (one side & no width)")
            return

        center_xs = 0.5 * (left_xs + right_xs)

        # 하단에서의 픽셀 간격으로 m/px 추정
        lw_pix_bottom = float((right_xs[-1] - left_xs[-1]))
        if lw_pix_bottom > 10:
            self.pix2m_x = self.lane_width_m / lw_pix_bottom

        # 7) 오프셋/헤딩오차 계산
        # 차량 중심 픽셀(ROI 좌표계): rw/2
        cx_pix = center_xs[-1]
        e_y_pix = (cx_pix - (rw * 0.5))  # 오른쪽이 + (이미지 x 오른쪽 증가)
        e_y_m = float(e_y_pix * (self.pix2m_x if self.pix2m_x else 0.02))

        # 중심선 기울기 -> 진행방향(이미지 좌표계에서 화면 아래로 y+)의 헤딩
        # 하단 2점으로 탄젠트
        y2 = rh - 1
        y1 = max(rh - 1 - self.look_y_pixels, 0)
        x2 = float(np.interp(y2, ys, center_xs))
        x1 = float(np.interp(y1, ys, center_xs))
        heading_img_rad = math.atan2((y2 - y1), (x2 - x1 + 1e-6))  # 이미지축 기준
        # 이미지 좌표를 차량 헤딩과 일치하도록 근사 변환: 화면 아래( +y )가 전방
        path_heading_deg = 90.0 - math.degrees(heading_img_rad)

        # 헤딩 오차(도로 - 차량): 도 단위 -> 라디안
        e_psi_deg = self.wrap_deg(path_heading_deg - self.yaw_deg)
        e_psi = math.radians(e_psi_deg)

        # EMA 스무딩
        self.ey_ema   = self.ema_alpha*e_y_m + (1-self.ema_alpha)*self.ey_ema
        self.epsi_ema = self.ema_alpha*e_psi + (1-self.ema_alpha)*self.epsi_ema

        # 8) Stanley 조향
        v = max(self.v_mps, 0.1)
        delta = self.epsi_ema + math.atan2(self.k_stanley * self.ey_ema, v)
        delta = max(-self.max_steer_rad, min(self.max_steer_rad, delta))

        # 9) 속도 PI 제어
        v_ref = self.target_speed_kmh / 3.6
        err = (v_ref - v)
        self.int_err = np.clip(self.int_err + err * 0.02, -5.0, 5.0)  # 적당한 적분 한계
        accel_cmd = self.speed_kp * err + self.speed_ki * self.int_err
        accel_cmd = float(np.clip(accel_cmd, -1.0, 1.0))

        brake = 0.0
        accel = 0.0
        if accel_cmd >= 0:
            accel = accel_cmd
        else:
            brake = -accel_cmd

        # 10) 명령 송신
        cmd = CtrlCmd()
        cmd.longlCmdType = 0  # Morai 기준: 0=가감속
        cmd.steering = float(delta)
        cmd.accel = float(accel)
        cmd.brake = float(brake)
        self.ctrl_pub.publish(cmd)

        # 11) 디버그 Path 게시
        self.publish_path(center_xs, ys, y0, rw, rh, frame)

        if self.debug_viz:
            dbg = self.draw_debug(roi, left_xs, right_xs, center_xs, ys, cx_pix)
            cv2.imshow("lane_debug", dbg)
            cv2.waitKey(1)

    # ================== 유틸/보조 ==================
    def extract_lane_points(self, binary):
        """이진 이미지에서 좌/우 차선 후보 픽셀 반환 (x,y) in ROI coords."""
        h, w = binary.shape[:2]
        # 좌우 분할
        mid = w // 2
        left_mask  = binary[:, :mid]
        right_mask = binary[:, mid:]

        # y-위치 가중치(하단 가중)
        ys, xs = np.where(left_mask > 0)
        left_pts = np.vstack([xs, ys]).T if len(xs) else np.empty((0,2), np.int32)
        ys, xs = np.where(right_mask > 0)
        if len(xs):
            right_pts = np.vstack([xs + mid, ys]).T
        else:
            right_pts = np.empty((0,2), np.int32)

        # 간단한 샘플링/정리
        if len(left_pts) > 0:
            left_pts = left_pts[left_pts[:,1].argsort()]
        if len(right_pts) > 0:
            right_pts = right_pts[right_pts[:,1].argsort()]
        return left_pts, right_pts

    def eval_poly(self, coef, ys):
        if coef is None:
            return None
        a,b,c = coef
        return a*ys*ys + b*ys + c

    def lane_width_pixels(self, xs, rh):
        """하단에서 좌/우 간격이 없을 때 기본 픽셀 폭 추정값(초깃값) 반환."""
        # 이전 프레임의 pix2m_x 있으면 역으로 추정
        if self.pix2m_x:
            return int(self.lane_width_m / self.pix2m_x)
        # 없으면 대략 화면 폭의 0.28배 정도를 가정(시야/카메라 FOV에 따라 튜닝)
        return None

    def publish_path(self, center_xs, ys, y0, rw, rh, frame):
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"  # rviz에서 보려면 고정 프레임에 맞추세요

        # 픽셀→m 변환(로컬 근사): x는 우측+, y는 전방+
        pix2m = self.pix2m_x if self.pix2m_x else 0.02
        for y_pix, x_pix in zip(ys, center_xs):
            # ROI 좌표를 차량 전방 좌표계로 근사 매핑
            xm = (x_pix - rw*0.5) * pix2m
            ym = (rh - y_pix) * pix2m  # 아래쪽이 가까움 → 앞쪽으로 변환
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = ym
            ps.pose.position.y = xm
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def publish_safe_stop(self, why):
        rospy.logwarn_throttle(1.0, why)
        # 속도 줄이고 스티어 0
        cmd = CtrlCmd()
        cmd.longlCmdType = 0
        cmd.steering = 0.0
        cmd.accel = 0.0
        cmd.brake = 0.2 if self.v_mps > 1.0 else 0.0
        self.ctrl_pub.publish(cmd)

    def draw_debug(self, roi, left_xs, right_xs, center_xs, ys, cx_pix):
        out = roi.copy()
        if left_xs is not None:
            for y,x in zip(ys.astype(int), left_xs.astype(int)):
                cv2.circle(out, (x,y), 2, (255,0,0), -1)
        if right_xs is not None:
            for y,x in zip(ys.astype(int), right_xs.astype(int)):
                cv2.circle(out, (x,y), 2, (0,0,255), -1)
        for y,x in zip(ys.astype(int), center_xs.astype(int)):
            cv2.circle(out, (x,y), 2, (0,255,0), -1)
        cv2.line(out, (int(roi.shape[1]*0.5), roi.shape[0]-1),
                      (int(cx_pix), roi.shape[0]-1), (0,255,255), 2)
        return out

    @staticmethod
    def wrap_deg(deg):
        while deg > 180: deg -= 360
        while deg < -180: deg += 360
        return deg

if __name__ == "__main__":
    try:
        LaneKeeperFrontView()
    except rospy.ROSInterruptException:
        pass

