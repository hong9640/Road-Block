#!/usr/bin/env python3
import rospy
from morai_msgs.msg import EgoVehicleStatus

latest_position = None

def ego_position_callback(msg):
    global latest_position
    latest_position = msg.position

def timer_callback(event):
    global latest_position
    if latest_position is not None:
        rospy.loginfo(f"Vehicle Position - x: {latest_position.x:.2f}, y: {latest_position.y:.2f}")

def listener():
    rospy.init_node('ego_position_listener', anonymous=True)
    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, ego_position_callback)
    
    rospy.Timer(rospy.Duration(1.0), timer_callback)  # 1초 간격 타이머 설정
    
    rospy.spin()

if __name__ == '__main__':
    listener()

