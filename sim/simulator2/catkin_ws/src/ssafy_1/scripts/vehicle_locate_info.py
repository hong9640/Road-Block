#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus

# 기본 URL 주소
BASE_URL = "ws://52.78.193.190:80/ws/vehicles"

def build_location_packet(vehicle_id: int, pos_x: float, pos_y: float) -> bytes:
    """
    [0-3]   uint32 LE : vehicle_id
    [4-7]   float32 LE: position_x
    [8-11]  float32 LE: position_y
    [12-27] 16 bytes  : hmac (현재는 NULL)
    총 28 bytes
    """
    data_part = struct.pack("<Iff", vehicle_id, pos_x, pos_y)
    hmac_bytes = b"\x00" * 16
    packet = data_part + hmac_bytes
    assert len(packet) == 28
    return packet

class LocationUpdater:
    def __init__(self):
        self.ego_topic = "/Ego_topic"
        self.latest_msg = None  # 가장 최근에 받은 메시지를 저장할 변수
        self.interval = 1.0     # 전송 간격 (초)

        # subscriber는 최신 메시지를 self.latest_msg에 저장하는 역할만 함
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_msg)

        # Timer가 self.interval 초마다 timed_send_callback 함수를 호출
        rospy.Timer(rospy.Duration(self.interval), self.timed_send_callback)

        rospy.loginfo(f"[LocationUpdater] Ready. Will send updates every {self.interval} seconds.")

    def send_location(self, url: str, pkt: bytes):
        try:
            ws = websocket.create_connection(url, timeout=2)
            ws.send_binary(pkt)
            rospy.loginfo(f"[LocationUpdater] Sent {len(pkt)} bytes to {url}")
            ws.close()
        except Exception as e:
            rospy.logwarn(f"[LocationUpdater] WebSocket send failed: {e}")

    def on_msg(self, msg: EgoVehicleStatus):
        # 콜백에서는 최신 메시지를 저장만 하고 아무것도 하지 않음
        self.latest_msg = msg

    def timed_send_callback(self, event=None):
        # 아직 메시지를 한 번도 받지 못했다면 아무것도 하지 않음
        if self.latest_msg is None:
            return

        # 저장된 최신 메시지에서 데이터 추출
        vehicle_id = self.latest_msg.unique_id
        pos_x = self.latest_msg.position.x
        pos_y = self.latest_msg.position.y

        # 데이터를 기반으로 패킷 생성
        pkt = build_location_packet(vehicle_id, pos_x, pos_y)

        # vehicle_id를 포함한 동적 URL 생성
        url = f"{BASE_URL}/{vehicle_id}/location"

        # 생성된 URL로 패킷 전송
        self.send_location(url, pkt)

if __name__ == "__main__":
    rospy.init_node("vehicle_location_sender", anonymous=False)
    LocationUpdater()
    rospy.spin() # 노드가 종료되지 않도록 유지
