#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus

URL = "ws://70.12.246.49:8000/ws/vehicles"  # 필요 시 /ws 로 교체
VEHICLE_TYPE = 0
CAR_NAME = "EGO_0"   # 10바이트 (부족하면 NULL padding, 넘치면 잘림)
MESSAGE_TYPE = 0xA0  # REGISTER_REQUEST

def build_packet(vehicle_id: int) -> bytes:
    """
    [0]     uint8     : message_type (0xA0)
    [1-4]   uint32 LE : vehicle_id
    [5]     uint8     : vehicle_type
    [6-15]  char[10]  : car_name (UTF-8, NULL padding)
    [16-31] 16 bytes  : hmac (여기서는 전부 NULL)
    총 32 bytes
    """
    # car_name → UTF-8 인코딩 후 10바이트 맞추기
    car_bytes = CAR_NAME.encode("utf-8")[:10].ljust(10, b"\x00")

    # hmac → 16바이트 NULL
    hmac_bytes = b"\x00" * 16

    # 패킷 생성 (<B I B 포맷: 1 + 4 + 1 = 6바이트)
    packet = struct.pack("<B I B", MESSAGE_TYPE, vehicle_id, VEHICLE_TYPE) + car_bytes + hmac_bytes
    assert len(packet) == 32
    return packet

class SimpleWSRegisterBin32:
    def __init__(self):
        self.ego_topic = "/Ego_topic"
        self.sent_once = False
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_msg)
        rospy.loginfo(f"[simple_ws_bin32] Ready, will connect only to {URL}")

    def send_binary_once(self, pkt: bytes):
        try:
            ws = websocket.create_connection(URL, timeout=5)
            ws.send_binary(pkt)
            rospy.loginfo(f"[simple_ws_bin32] Sent {len(pkt)} bytes to {URL}")
            rospy.loginfo("[simple_ws_bin32] HEX: " + pkt.hex(" "))
            ws.close()
        except Exception as e:
            rospy.logerr(f"[simple_ws_bin32] WebSocket send failed: {e}")

    def on_msg(self, msg: EgoVehicleStatus):
        if self.sent_once:
            return
        self.sent_once = True

        vehicle_id = getattr(msg, "unique_id", 0)
        rospy.loginfo(f"[simple_ws_bin32] vehicle_id from ROS = {vehicle_id}")

        pkt = build_packet(int(vehicle_id))
        self.send_binary_once(pkt)

        rospy.signal_shutdown("done")

if __name__ == "__main__":
    rospy.init_node("simple_ws_register_bin32", anonymous=False)
    SimpleWSRegisterBin32()
    rospy.spin()

