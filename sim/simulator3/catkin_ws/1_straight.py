#!/usr/bin/env python3
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus

class StraightDrivingNode:
    def __init__(self):
        rospy.init_node('straight_driving_node')

        self.ctrl_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        rospy.Subscriber('/EgoVehicleStatus', EgoVehicleStatus, self.status_callback)

        self.current_velocity = 0.0
        self.rate = rospy.Rate(10)  # 10Hz

    def status_callback(self, msg):
        self.current_velocity = msg.velocity.x
        rospy.loginfo(f"Current vehicle speed: {self.current_velocity:.2f} m/s")

    def run(self):
        while not rospy.is_shutdown():
            cmd = CtrlCmd()
            cmd.longlCmdType = 2    # 속도 제어 모드 (실제 문서 확인 필요)
            cmd.accel = 0.5        # 가속도 값 10
            cmd.brake = 0.0         # 브레이크 밟지 않음
            cmd.steering = 0.0      # 조향 없음(직진)
            cmd.velocity = 60.0     # 목표 속도 60 km/h
            cmd.acceleration = 0.0  # 추가 가속도 필드 (필요시 설정)

            self.ctrl_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = StraightDrivingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

