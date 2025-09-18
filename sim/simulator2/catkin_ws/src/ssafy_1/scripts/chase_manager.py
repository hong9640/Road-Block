#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import websocket
import threading
import struct
import hmac
import hashlib
import os
from dotenv import load_dotenv
from std_msgs.msg import String
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, VehicleCollisionData

# --- 설정 ---
BASE_URL_DEFAULT = "ws://52.78.193.190:8080/ws/vehicles"
ENV_PATH_DEFAULT = "/home/ubuntu/S13P21A507/sim/simulator2/catkin_ws/.env"
COMPLETE_DESTROYED_THRESHOLD = 2 # 완전 파손으로 간주할 충돌 횟수

class ChaseManager:
    """
    백엔드 서버와 통신하며 추격 시나리오의 시작과 끝을 관리하는 노드.
    - 웹소켓을 통해 '도주 시작' 이벤트를 수신.
    - 도주 차량의 파손 상태를 감시하여 '검거 완료' 이벤트를 송신.
    - 추격 제어 노드(LaneFollowingPursuitNode2)에 시작/정지 명령 전달.
    """
    def __init__(self):
        rospy.init_node("chase_manager", anonymous=True)

        # --- ROS 파라미터 로드 ---
        self.base_url = rospy.get_param("~base_url", BASE_URL_DEFAULT)
        env_path = rospy.get_param("~env_path", ENV_PATH_DEFAULT)
        
        # --- 환경 변수에서 HMAC 키 로드 ---
        load_dotenv(env_path)
        secret_key_str = os.getenv("HMAC_SECRET_KEY")
        if not secret_key_str:
            rospy.logerr(f"HMAC_SECRET_KEY 환경 변수를 찾을 수 없습니다. ({env_path})")
            rospy.signal_shutdown("HMAC_SECRET_KEY not set")
            return
        self.secret_key = secret_key_str.encode("utf-8")

        # --- 상태 변수 ---
        self.catcher_id = None
        self.runner_id = None
        self.chase_active = False
        self.fugitive_collision_count = 0
        self.in_fugitive_collision = False
        self.catch_event_sent = False

        # --- ROS Publisher ---
        self.chase_status_pub = rospy.Publisher("/chase_status", String, queue_size=1)
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1, latch=True)

        # --- ROS Subscriber ---
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        rospy.Subscriber("/fugitive_collision", VehicleCollisionData, self.fugitive_collision_callback)

        # --- 웹소켓 리스너 스레드 시작 ---
        self.ws_thread = threading.Thread(target=self._ws_listener_thread)
        self.ws_thread.daemon = True
        self.ws_thread.start()

        rospy.loginfo("Chase Manager 시작. 백엔드 이벤트 수신 대기 중...")

    def _calculate_hmac(self, data: bytes) -> bytes:
        return hmac.new(self.secret_key, data, hashlib.sha256).digest()[:16]

    def ego_status_callback(self, msg):
        if self.catcher_id is None and msg.unique_id != 0:
            self.catcher_id = msg.unique_id
            rospy.loginfo(f"추격 차량 ID 확인: {self.catcher_id}")

    def fugitive_collision_callback(self, msg):
        if not self.chase_active or self.catch_event_sent:
            return

        has_collision = len(msg.collision_object) > 0
        if has_collision and not self.in_fugitive_collision:
            self.fugitive_collision_count += 1
            rospy.loginfo(f"도주 차량 충돌 감지! (누적: {self.fugitive_collision_count})")

            if self.fugitive_collision_count >= COMPLETE_DESTROYED_THRESHOLD:
                rospy.loginfo("도주 차량 완전 파손! 검거 성공!")
                self._send_catch_event()
                self.catch_event_sent = True
        
        self.in_fugitive_collision = has_collision
        
    def _ws_listener_thread(self):
        """백엔드 서버에 연결하여 '도주 시작' 이벤트를 기다리는 스레드."""
        ws_url = self.base_url 
        while not rospy.is_shutdown():
            try:
                rospy.loginfo(f"도주 시작 이벤트 웹소켓 연결 시도: {ws_url}")
                ws = websocket.create_connection(ws_url, timeout=10)
                rospy.loginfo("도주 시작 이벤트 웹소켓 연결 성공.")
                while not rospy.is_shutdown():
                    data = ws.recv()
                    if len(data) == 21:
                        msg_type, runner_id = struct.unpack("<BI", data[:5])
                        if msg_type == 0xFF and not self.chase_active:
                            self.runner_id = runner_id
                            self.chase_active = True
                            self.chase_status_pub.publish("START")
                            rospy.loginfo(f"===== 도주 시작 이벤트 수신! 도주차량 ID: {self.runner_id} =====")

            except Exception as e:
                rospy.logwarn(f"웹소켓 에러: {e}. 5초 후 재연결...")
                rospy.sleep(5)

    def _send_catch_event(self):
        """도주 차량 검거 성공 이벤트를 백엔드에 전송합니다."""
        if self.catcher_id is None or self.runner_id is None:
            rospy.logwarn("검거 이벤트 전송 실패: ID 정보가 부족합니다.")
            return

        header9 = struct.pack("<BII", 0xFE, self.catcher_id, self.runner_id)
        packet = header9 + self._calculate_hmac(header9)

        ws_url = self.base_url.replace("/vehicles", "/events")
        
        try:
            rospy.loginfo(f"검거 완료 이벤트 전송 -> {ws_url}")
            ws = websocket.create_connection(ws_url, timeout=3)
            ws.send_binary(packet)
            ws.close()
            
            self._stop_pursuit()

        except Exception as e:
            rospy.logerr(f"검거 이벤트 전송 실패: {e}")

    def _stop_pursuit(self):
        rospy.loginfo("추격 중지 명령 발행.")
        self.chase_status_pub.publish("STOP")

        stop_cmd = CtrlCmd()
        stop_cmd.longlCmdType = 1
        stop_cmd.accel = 0.0
        stop_cmd.brake = 1.0
        stop_cmd.steering = 0.0
        self.ctrl_pub.publish(stop_cmd)


if __name__ == "__main__":
    try:
        ChaseManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
