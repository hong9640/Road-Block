#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import hmac
import hashlib
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus
from dotenv import load_dotenv

# -----------------------------
# 설정
# -----------------------------
BASE_URL = "ws://52.78.193.190:8080/ws/vehicles"
ENV_PATH = "/home/ubuntu/S13P21A507/sim/simulator2/catkin_ws/.env"  # 절대경로

# -----------------------------
# HMAC KEY 로드 (.env 절대경로에서)
# -----------------------------
load_dotenv(ENV_PATH)  # .env 파일 읽기
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError(f"HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일({ENV_PATH})을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode("utf-8")

def _calculate_hmac16(data: bytes) -> bytes:
    """주어진 데이터(12바이트)에 대해 HMAC-SHA256 계산 후 앞 16바이트만 반환."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

def build_location_packet(vehicle_id: int, pos_x: float, pos_y: float) -> bytes:
    """
    [0-3]   uint32 LE : vehicle_id
    [4-7]   float32 LE: position_x
    [8-11]  float32 LE: position_y
    [12-27] 16 bytes  : hmac(HMAC-SHA256([0:12])의 앞 16바이트)
    총 28 bytes
    """
    data12 = struct.pack("<Iff", vehicle_id, pos_x, pos_y)
    hmac16 = _calculate_hmac16(data12)
    packet = data12 + hmac16
    assert len(packet) == 28
    return packet

class LocationUpdater:
    def __init__(self):
        self.ego_topic = "/Ego_topic"
        self.latest_msg = None   # 가장 최근에 받은 메시지를 저장
        self.interval = 1.0      # 전송 간격(초)

        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_msg)
        rospy.Timer(rospy.Duration(self.interval), self.timed_send_callback)

        rospy.loginfo(f"[LocationUpdater] Ready. Will send updates every {self.interval} seconds.")

    def send_location(self, url: str, pkt: bytes):
        try:
            ws = websocket.create_connection(url, timeout=2)
            ws.send_binary(pkt)
            rospy.loginfo(f"[LocationUpdater] Sent {len(pkt)} bytes to {url}")
            # 필요 시 HEX 로깅
            # rospy.loginfo("[LocationUpdater] HEX: " + pkt.hex(" "))
            ws.close()
        except Exception as e:
            rospy.logwarn(f"[LocationUpdater] WebSocket send failed: {e}")

    def on_msg(self, msg: EgoVehicleStatus):
        self.latest_msg = msg  # 최신 메시지 캐시

    def timed_send_callback(self, event=None):
        if self.latest_msg is None:
            return

        vehicle_id = int(self.latest_msg.unique_id)
        pos_x = float(self.latest_msg.position.x)
        pos_y = float(self.latest_msg.position.y)

        pkt = build_location_packet(vehicle_id, pos_x, pos_y)
        url = f"{BASE_URL}/{vehicle_id}/location"

        self.send_location(url, pkt)

if __name__ == "__main__":
    rospy.init_node("vehicle_location_sender", anonymous=False)
    LocationUpdater()
    rospy.spin()

