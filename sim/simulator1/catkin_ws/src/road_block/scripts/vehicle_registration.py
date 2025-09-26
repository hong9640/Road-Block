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

# --- 설정 ---
URL_DEFAULT = "wss://j13a507.p.ssafy.io/ws/vehicles"
VEHICLE_TYPE_DEFAULT = 0 # 0: 경찰, 1: 도둑
CAR_NAME_DEFAULT = "POLICE_CAR"
MESSAGE_TYPE = 0xA0
ENV_PATH_DEFAULT = "/home/ubuntu/S13P21A507/sim/simulator1/catkin_ws/.env"

class VehicleRegistrar:
    def __init__(self):
        rospy.init_node("vehicle_registrar", anonymous=True)

        # --- ROS 파라미터 로드 ---
        self.url = rospy.get_param("~url", URL_DEFAULT)
        self.vehicle_type = rospy.get_param("~vehicle_type", VEHICLE_TYPE_DEFAULT)
        self.car_name = rospy.get_param("~car_name", CAR_NAME_DEFAULT)
        env_path = rospy.get_param("~env_path", ENV_PATH_DEFAULT)
        
        self.vehicle_id_override = int(rospy.get_param("~vehicle_id_override", 0))
        # --- HMAC 키 로드 ---
        load_dotenv(env_path)
        secret_key_str = os.getenv("HMAC_SECRET_KEY")
        if not secret_key_str:
            rospy.logerr(f"HMAC_SECRET_KEY를 찾을 수 없습니다. ({env_path})")
            rospy.signal_shutdown("HMAC_SECRET_KEY not set")
            return
        self.secret_key = secret_key_str.encode("utf-8")
        
        self.sent_once = False
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.on_ego_status)
        #rospy.loginfo(f"차량 등록 대기 중: [이름: {self.car_name}, 타입: {self.vehicle_type}]")

    def _calculate_hmac(self, data: bytes) -> bytes:
        return hmac.new(self.secret_key, data, hashlib.sha256).digest()[:16]

    def build_packet(self, vehicle_id: int) -> bytes:
        car_bytes = self.car_name.encode("utf-8")[:10].ljust(10, b"\x00")
        head16 = struct.pack("<B I B", MESSAGE_TYPE, vehicle_id, self.vehicle_type) + car_bytes
        hmac_bytes = self._calculate_hmac(head16)
        return head16 + hmac_bytes

    def on_ego_status(self, msg: EgoVehicleStatus):
        if self.sent_once:
            return
        
        if self.vehicle_id_override != 0:
            vehicle_id = self.vehicle_id_override
        else:
            vehicle_id = msg.unique_id
        
        #rospy.loginfo(f"차량 ID ({vehicle_id}) 수신. 등록 패킷 전송 시도...")
        
        pkt = self.build_packet(vehicle_id)
        try:
            ws = websocket.create_connection(self.url, timeout=5)
            ws.send_binary(pkt)
            #rospy.loginfo(f"차량 등록 성공! {len(pkt)} bytes 전송 -> {self.url}")
            ws.close()
            self.sent_once = True
            rospy.signal_shutdown("차량 등록 완료. 노드 종료.")
        except Exception as e:
            rospy.logerr(f"웹소켓 전송 실패: {e}. 5초 후 재시도...")
            rospy.sleep(5) # 재시도를 위해 종료하지 않음

if __name__ == "__main__":
    try:
        VehicleRegistrar()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
