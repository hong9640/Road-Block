#!/usr/bin/env python3
import rospy
import math
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class FollowEgo2Node:
    def __init__(self):
        rospy.init_node('follow_ego2_node')

        self.ctrl_pub = rospy.Publisher('/Ego-3/ctrl_cmd', CtrlCmd, queue_size=1)

        rospy.Subscriber('/Ego-3/Ego_topic', EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber('/Ego-2/Ego_topic', EgoVehicleStatus, self.target_status_callback)

        # Ego 차량 상태
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0  # rad
        self.current_velocity = 0.0  # m/s

        # 타겟 차량 상태
        self.target_x = None
        self.target_y = None

        # 제어 파라미터
        self.TARGET_DISTANCE = 8.0       # m
        self.MAX_VELOCITY = 11.11        # 40 kph ≈ 11.11 m/s
        self.STEERING_GAIN = 1.8
        self.VELOCITY_KP = 0.8           # 속도 제어 gain (조금 키움)

        self.rate = rospy.Rate(10)
        self.last_print_time = rospy.get_time()

    def ego_status_callback(self, msg):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y
        # heading 은 degree → rad 변환 필요
        self.ego_yaw = math.radians(msg.heading)
        self.current_velocity = msg.velocity.x  # m/s

    def target_status_callback(self, msg):
        self.target_x = msg.position.x
        self.target_y = msg.position.y

    def run(self):
        while not rospy.is_shutdown():
            if self.target_x is None or self.target_y is None:
                rospy.loginfo("타겟 차량의 위치를 기다리는 중...")
                self.rate.sleep()
                continue

            dx = self.target_x - self.ego_x
            dy = self.target_y - self.ego_y
            distance = math.sqrt(dx**2 + dy**2)

            # 목표 각도 (라디안)
            target_angle = math.atan2(dy, dx)

            # 조향 오차
            angle_diff = target_angle - self.ego_yaw
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # 조향 제어
            steering_cmd = self.STEERING_GAIN * angle_diff
            steering_cmd = max(min(steering_cmd, 1.0), -1.0)

            # 속도 제어
            velocity_error = distance - self.TARGET_DISTANCE
            target_velocity = self.VELOCITY_KP * velocity_error

            # 안전 범위 제한
            target_velocity = max(0.0, min(target_velocity, self.MAX_VELOCITY))

            # 제어 명령 생성
            cmd = CtrlCmd()
            cmd.longlCmdType = 2  # velocity mode
            cmd.steering = steering_cmd
            cmd.velocity = target_velocity
            self.ctrl_pub.publish(cmd)

            # 디버깅 출력
            current_time = rospy.get_time()
            if current_time - self.last_print_time >= 1.0:
                rospy.loginfo(f"[EGO] ({self.ego_x:.2f}, {self.ego_y:.2f}, {math.degrees(self.ego_yaw):.1f}°) "
                              f"| [TARGET] ({self.target_x:.2f}, {self.target_y:.2f}) "
                              f"| Dist: {distance:.2f} m | V_cmd: {target_velocity:.2f} m/s | Steer: {steering_cmd:.2f}")
                self.last_print_time = current_time

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FollowEgo2Node()
        node.run()
    except rospy.ROSInterruptException:
        pass
