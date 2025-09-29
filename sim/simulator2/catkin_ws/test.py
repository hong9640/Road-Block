#!/usr/bin/env python3
import rospy
import math
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class FollowEgo2Node:
    def __init__(self):
        rospy.init_node('follow_ego3_node')

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
        self.TARGET_DISTANCE = 8.0
        self.MAX_VELOCITY = 11.11       # 40 kph
        self.STEERING_GAIN = 1.8
        self.VELOCITY_KP = 0.8

        self.rate = rospy.Rate(10)
        self.last_print_time = rospy.get_time()

    def ego_status_callback(self, msg):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y
        self.ego_yaw = math.radians(msg.heading)  # deg -> rad
        self.current_velocity = msg.velocity.x

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

            target_angle = math.atan2(dy, dx)
            angle_diff = (target_angle - self.ego_yaw + math.pi) % (2 * math.pi) - math.pi

            steering_cmd = max(min(self.STEERING_GAIN * angle_diff, 1.0), -1.0)

            target_velocity = max(0.0, min(self.VELOCITY_KP * (distance - self.TARGET_DISTANCE), self.MAX_VELOCITY))

            cmd = CtrlCmd()
            cmd.longlCmdType = 2  # velocity mode
            cmd.steering = steering_cmd
            cmd.velocity = target_velocity
            # accel/brake/acceleration 필드는 velocity mode에서는 보통 무시됨

            self.ctrl_pub.publish(cmd)

            now = rospy.get_time()
            if now - self.last_print_time >= 1.0:
                rospy.loginfo(
                    f"[EGO] ({self.ego_x:.2f}, {self.ego_y:.2f}, {math.degrees(self.ego_yaw):.1f}°) "
                    f"| [TARGET] ({self.target_x:.2f}, {self.target_y:.2f}) "
                    f"| Dist: {distance:.2f} m | V_cmd: {target_velocity:.2f} m/s | Steer: {steering_cmd:.2f}"
                )
                self.last_print_time = now

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = FollowEgo2Node()
        node.run()
    except rospy.ROSInterruptException:
        pass

