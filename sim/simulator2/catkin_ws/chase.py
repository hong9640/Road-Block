#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from morai_msgs.msg import EgoVehicleStatus, CtrlCmd

def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class FollowEgo2Node:
    def __init__(self):
        rospy.init_node("follow_ego2_node")

        # 제어 파라미터
        self.TARGET_DISTANCE = 8.0       # m
        self.MAX_VELOCITY = 11.11        # 40 km/h
        self.KP_DIST = 0.8
        self.KP_STEER = 1.6
        self.MAX_STEER = 0.6             # rad
        self.LOST_TIMEOUT = 1.0          # sec

        # Ego-3 상태
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_speed = 0.0

        # Ego-2 상태 (타겟)
        self.target_x = None
        self.target_y = None
        self.last_target_stamp = None

        # publisher & subscribers
        self.ctrl_pub = rospy.Publisher("/Ego-3/ctrl_cmd", CtrlCmd, queue_size=1)
        rospy.Subscriber("/Ego-3/Ego_topic", EgoVehicleStatus, self.ego_cb)
        rospy.Subscriber("/Ego-2/Ego_topic", EgoVehicleStatus, self.target_cb)

        self.rate = rospy.Rate(20)  # Hz

    def ego_cb(self, msg: EgoVehicleStatus):
        self.ego_x = msg.position.x
        self.ego_y = msg.position.y
        self.ego_yaw = msg.heading
        self.ego_speed = math.hypot(msg.velocity.x, msg.velocity.y)

    def target_cb(self, msg: EgoVehicleStatus):
        self.target_x = msg.position.x
        self.target_y = msg.position.y
        self.last_target_stamp = rospy.Time.now()

    def compute_cmd(self):
        cmd = CtrlCmd()
        cmd.longlCmdType = 2  # 속도/조향 직접 제어 모드

        if self.target_x is None or self.target_y is None:
            cmd.velocity = 0.0
            cmd.steering = 0.0
            return cmd

        if (rospy.Time.now() - self.last_target_stamp).to_sec() > self.LOST_TIMEOUT:
            cmd.velocity = 0.0
            cmd.steering = 0.0
            return cmd

        dx = self.target_x - self.ego_x
        dy = self.target_y - self.ego_y
        dist = math.hypot(dx, dy)

        target_bearing = math.atan2(dy, dx)
        heading_error = normalize_angle(target_bearing - self.ego_yaw)

        # 속도 제어
        dist_err = dist - self.TARGET_DISTANCE
        vel_cmd = self.KP_DIST * dist_err
        vel_cmd = max(0.0, min(self.MAX_VELOCITY, vel_cmd))

        # 조향 제어
        steer_cmd = self.KP_STEER * heading_error
        steer_cmd = max(-self.MAX_STEER, min(self.MAX_STEER, steer_cmd))

        cmd.velocity = vel_cmd
        cmd.steering = steer_cmd
        return cmd

    def spin(self):
        while not rospy.is_shutdown():
            cmd = self.compute_cmd()
            self.ctrl_pub.publish(cmd)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = FollowEgo2Node()
        node.spin()
    except rospy.ROSInterruptException:
        pass

