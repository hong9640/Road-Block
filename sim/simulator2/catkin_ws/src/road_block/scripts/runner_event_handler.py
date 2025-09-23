#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import hmac
import hashlib
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus, CollisionData
from dotenv import load_dotenv

# =============================
# 설정
# =============================
BASE_URL_DEFAULT = "ws://52.78.193.190:8080/ws" # 기본 URL을 /ws로 변경
EGO_TOPIC_DEFAULT = "/Ego_topic"
COLLISION_TOPIC_DEFAULT = "/CollisionData"

# =============================
# HMAC KEY 로드
# =============================
ENV_PATH = "/home/ubuntu/S13P21A507/sim/simulator2/catkin_ws/.env"
load_dotenv(ENV_PATH)
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError(f"HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일({ENV_PATH})을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode("utf-8")


def _hmac16(data: bytes) -> bytes:
    """주어진 데이터에 대해 HMAC-SHA256 계산 후 앞 16바이트만 반환."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]


# =============================
# 메인 클래스
# =============================
class RunnerCatchHandler:
    def __init__(self):
        rospy.init_node("runner_catch_handler", anonymous=False)

        # ROS 파라미터 로드
        self.base_url = rospy.get_param("~base_url", BASE_URL_DEFAULT)
        self.runner_id = int(rospy.get_param("~vehicle_id_override", 0))

        # 상태 변수
        self.is_caught = False # 검거 이벤트가 한 번만 발생하도록 하는 플래그

        # Subscriber
        rospy.Subscriber(EGO_TOPIC_DEFAULT, EgoVehicleStatus, self.on_ego_status)
        rospy.Subscriber(COLLISION_TOPIC_DEFAULT, CollisionData, self.on_collision)

        rospy.loginfo(f"[RunnerCatchHandler] Ready. Runner ID: {self.runner_id}")

    def on_ego_status(self, msg: EgoVehicleStatus):
        """런치파일에서 ID를 지정하지 않았을 경우, 메시지에서 자신의 ID를 설정"""
        if self.runner_id == 0:
            self.runner_id = msg.unique_id
            rospy.loginfo(f"[RunnerCatchHandler] Runner ID set to {self.runner_id}")

    def on_collision(self, msg: CollisionData):
        """충돌 발생 시 호출되는 콜백 함수"""
        if self.is_caught or not msg.collision_object:
            return

        catcher_id = msg.collision_object[0].unique_id

        rospy.loginfo(f"!!! 도주 차량(ID: {self.runner_id})이 경찰차(ID: {catcher_id})와 충돌 !!!")
        rospy.loginfo("'검거 성공(0xFE)' 이벤트를 서버로 전송합니다.")

        # 검거 이벤트 패킷 생성
        packet = self.build_catch_packet(0xFE, int(catcher_id), int(self.runner_id))

        # /ws/events 로 이벤트 전송
        event_url = f"{self.base_url}/events"
        self.send_event(event_url, packet)

        # 검거 상태로 변경하여 중복 전송 방지
        self.is_caught = True
        
        # 역할을 다했으므로 노드를 종료
        rospy.loginfo("검거 이벤트 전송 완료. 노드를 종료합니다.")
        rospy.signal_shutdown("Caught by police.")

    def build_catch_packet(self, message_type: int, catcher_id: int, runner_id: int) -> bytes:
        """검거 이벤트(0xFE) 패킷을 생성합니다."""
        header9 = struct.pack("<BII", message_type, catcher_id, runner_id)
        return header9 + _hmac16(header9)

    def send_event(self, url: str, pkt: bytes):
        """웹소켓으로 이벤트를 전송합니다."""
        try:
            ws = websocket.create_connection(url, timeout=3)
            ws.send_binary(pkt)
            ws.close()
        except Exception as e:
            rospy.logerr(f"이벤트 전송 실패: {e}")


if __name__ == "__main__":
    try:
        RunnerCatchHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
