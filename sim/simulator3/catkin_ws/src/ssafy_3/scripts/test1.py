#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def ang_norm(a):       return (a + math.pi) % (2*math.pi) - math.pi

class FollowEgoNode:
    """
    Ego-3가 Ego-2를 따라가는 단순 추종 컨트롤러
    - 스티어링: 헤딩 오차(P 제어)
    - 속도: 차간 거리 오차(P + 타겟 속도 feedforward)
    - 타겟 유실/근접/과도한 헤딩오차에 대한 가드 포함
    """
    def __init__(self):
        rospy.init_node('follow_ego_node')

        # ----------------- 파라미터 -----------------
        self.self_ns    = rospy.get_param('~self_ns',   'Ego-3')  # 내가 조종할 이고
        self.target_ns  = rospy.get_param('~target_ns', 'Ego-2')  # 따라갈 타겟
        self.rate_hz    = float(rospy.get_param('~rate_hz', 20.0))

        # 거리/속도 관련
        self.target_gap_m   = float(rospy.get_param('~target_gap_m', 8.0))   # 유지 거리
        self.max_vel_ms     = float(rospy.get_param('~max_vel_ms',   11.11)) # 40km/h
        self.min_vel_ms     = float(rospy.get_param('~min_vel_ms',   0.0))
        self.kp_gap         = float(rospy.get_param('~kp_gap',       0.8))   # 거리 P게인
        self.ff_gain        = float(rospy.get_param('~ff_gain',      0.5))   # 타겟 속도 feedforward
        self.max_accel_ms2  = float(rospy.get_param('~max_accel_ms2',1.5))   # 가감속 제한

        # 조향 관련
        self.kp_yaw         = float(rospy.get_param('~kp_yaw',       1.8))   # 헤딩 P게인
        self.max_steer      = float(rospy.get_param('~max_steer',    1.0))   # [-1,1]
        self.max_steer_rate = float(rospy.get_param('~max_steer_rate',0.8))  # /s (옵션)

        # 가드/유실 대응
        self.close_stop_m   = float(rospy.get_param('~close_stop_m', 3.0))   # 너무 가까우면 정지
        self.lost_timeout_s = float(rospy.get_param('~lost_timeout_s', 1.0))
        self.max_yaw_err_rad= float(rospy.get_param('~max_yaw_err_rad', math.radians(90)))
        # --------------------------------------------

        # 퍼블리셔/서브스크라이버
        self.ctrl_pub = rospy.Publisher(f'/{self.self_ns}/ctrl_cmd', CtrlCmd, queue_size=1)
        rospy.Subscriber(f'/{self.self_ns}/Ego_topic',   EgoVehicleStatus, self.ego_cb,    queue_size=1)
        rospy.Subscriber(f'/{self.target_ns}/Ego_topic', EgoVehicleStatus, self.target_cb, queue_size=1)

        # 상태
        self.ex = self.ey = 0.0
        self.eyaw = 0.0
        self.e_v = 0.0

        self.tx = self.ty = None
        self.t_v = 0.0
        self.t_last_stamp = None

        self._last_vel_cmd = 0.0
        self._last_steer   = 0.0
        self._last_loop    = rospy.Time.now().to_sec()
        self._last_log     = self._last_loop

        self.rate = rospy.Rate(self.rate_hz)

    # 내 차량 상태
    def ego_cb(self, msg:EgoVehicleStatus):
        self.ex = msg.position.x
        self.ey = msg.position.y
        self.eyaw = math.radians(msg.heading)  # deg→rad
        self.e_v = msg.velocity.x              # m/s (x 전진)

    # 타겟 상태
    def target_cb(self, msg:EgoVehicleStatus):
        self.tx = msg.position.x
        self.ty = msg.position.y
        self.t_v = msg.velocity.x
        self.t_last_stamp = rospy.Time.now()

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt  = max(1e-3, now - self._last_loop)
            self._last_loop = now

            have_target = (self.tx is not None and self.ty is not None)
            if not have_target or (self.t_last_stamp and (rospy.Time.now() - self.t_last_stamp).to_sec() > self.lost_timeout_s):
                # 타겟 유실 → 안전 정지
                self.publish_cmd(steer=self._last_steer, vel=0.0)
                self.rate.sleep()
                continue

            # 기하
            dx = self.tx - self.ex
            dy = self.ty - self.ey
            dist = math.hypot(dx, dy)

            tgt_yaw = math.atan2(dy, dx)
            yaw_err = ang_norm(tgt_yaw - self.eyaw)

            # 스티어링 (P + 제한 + slew-rate 제한)
            steer = clamp(self.kp_yaw * yaw_err, -self.max_steer, self.max_steer)
            max_step = self.max_steer_rate * dt
            steer = clamp(steer, self._last_steer - max_step, self._last_steer + max_step)

            # 속도 명령: 거리 오차 P + 타겟 속도 feedforward
            gap_err = dist - self.target_gap_m
            vel_cmd_raw = self.kp_gap * gap_err + self.ff_gain * max(0.0, self.t_v)

            # 근접/뒤돌아봄 가드
            if dist <= self.close_stop_m or abs(yaw_err) > self.max_yaw_err_rad:
                vel_cmd_raw = 0.0

            # 속도 한계 + 가감속 제한(slew)
            vel_cmd = clamp(vel_cmd_raw, self.min_vel_ms, self.max_vel_ms)
            dv = clamp(vel_cmd - self._last_vel_cmd, -self.max_accel_ms2 * dt, self.max_accel_ms2 * dt)
            vel_cmd = self._last_vel_cmd + dv

            self.publish_cmd(steer=steer, vel=vel_cmd)

            if now - self._last_log > 1.0:
                rospy.loginfo(f"[{self.self_ns}] pos=({self.ex:.1f},{self.ey:.1f}) v={self.e_v:.2f} "
                              f"| tgt=({self.tx:.1f},{self.ty:.1f}) tv={self.t_v:.2f} "
                              f"| dist={dist:.2f} yaw_err={math.degrees(yaw_err):.1f} "
                              f"| v_cmd={vel_cmd:.2f} steer={steer:.2f}")
                self._last_log = now

            self._last_vel_cmd = vel_cmd
            self._last_steer   = steer
            self.rate.sleep()

    def publish_cmd(self, steer:float, vel:float):
        cmd = CtrlCmd()
        cmd.longlCmdType = 2          # velocity mode
        cmd.steering     = clamp(steer, -self.max_steer, self.max_steer)
        cmd.velocity     = max(0.0, vel)
        # accel/brake/acceleration는 velocity mode에서 보통 무시됨
        self.ctrl_pub.publish(cmd)

if __name__ == '__main__':
    try:
        node = FollowEgoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

