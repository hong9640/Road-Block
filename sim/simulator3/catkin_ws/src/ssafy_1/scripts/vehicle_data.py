#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import hmac
import hashlib
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus
from dotenv import load_dotenv   # 추가

# -----------------------------
# 설정
# -----------------------------
URL = "ws://52.78.193.190:8080/ws/vehicles"
VEHICLE_TYPE = 0
CAR_NAME = "EGO_0"
MESSAGE_TYPE = 0xA0
ENV_PATH = "/home/ubuntu/S13P21A507/sim/simulator2/catkin_ws/.env"  # 절대경로

# -----------------------------
# HMAC KEY 로드 (.env 절대경로에서)
# -----------------------------
load_dotenv(ENV_PATH)  # .env 파일 읽기

SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError(f"HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일({ENV_PATH})을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode("utf-8")

def _calculate_hmac(data: bytes) -> bytes:
    """주어진 데이터로 HMAC-SHA256 값을 계산 (앞 16바이트)."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

def build_packet(vehicle_id: int) -> bytes:
    """32바이트 등록 패킷 생성"""
    car_bytes = CAR_NAME.encode("utf-8")[:10].ljust(10, b"\x00")
    head16 = struct.pack("<B I B", MESSAGE_TYPE, vehicle_id, VEHICLE_TYPE) + car_bytes
    assert len(head16) == 16

    hmac_bytes = _calculate_hmac(head16)
    packet = head16 + hmac_bytes
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
