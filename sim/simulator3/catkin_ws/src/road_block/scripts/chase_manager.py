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
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, CollisionData

# --- 설정 ---
BASE_URL_DEFAULT = "wss://j13a507.p.ssafy.io/ws/vehicles"
ENV_PATH_DEFAULT = "/home/ubuntu20/S13P21A507/sim/simulator3/catkin_ws/.env"

class ChaseManager:
    """
    웹소켓 채널을 감시하며 추격 시나리오의 시작과 끝을 관리하는 노드.
    """
    def __init__(self):
        rospy.init_node("chase_manager", anonymous=True)

        self.base_url = rospy.get_param("~base_url", BASE_URL_DEFAULT)
        env_path = rospy.get_param("~env_path", ENV_PATH_DEFAULT)
        
        load_dotenv(env_path)
        secret_key_str = os.getenv("HMAC_SECRET_KEY")
        if not secret_key_str:
            rospy.logerr(f"HMAC_SECRET_KEY 환경 변수를 찾을 수 없습니다. ({env_path})")
            rospy.signal_shutdown("HMAC_SECRET_KEY not set")
            return
        self.secret_key = secret_key_str.encode("utf-8")

        # --- 상태 변수 ---
        self.catcher_id = None
        self.runner_id = None # 추격 시작 시 ID가 할당됨
        self.chase_active = False

        # --- ROS Publisher/Subscriber ---
        self.chase_status_pub = rospy.Publisher("/chase_status", String, queue_size=1, latch=True)
        self.ctrl_pub = rospy.Publisher("/ctrl_cmd", CtrlCmd, queue_size=1, latch=True)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.ego_status_callback)
        # 도주차량 충돌 구독 로직은 제거됨

        # --- 두 개의 웹소켓 리스너 스레드 시작 ---
        # 스레드 1: '/ws/vehicles'에서 추격 시작(0xFF) 이벤트 수신
        self.run_event_thread = threading.Thread(target=self._ws_run_event_listener_thread)
        self.run_event_thread.daemon = True
        self.run_event_thread.start()

        # 스레드 2: '/ws/events'에서 검거 성공(0xFC) 방송 수신
        self.catch_event_thread = threading.Thread(target=self._ws_catch_broadcast_listener_thread)
        self.catch_event_thread.daemon = True
        self.catch_event_thread.start()

        rospy.loginfo("Chase Manager 시작. 백엔드 이벤트 수신 대기 중...")

    def _calculate_hmac(self, data: bytes) -> bytes:
        return hmac.new(self.secret_key, data, hashlib.sha256).digest()[:16]

    def ego_status_callback(self, msg: EgoVehicleStatus):
        if self.catcher_id is None and msg.unique_id != 0:
            self.catcher_id = msg.unique_id
            rospy.loginfo(f"추격 차량 ID 확인: {self.catcher_id}")
    
    def _ws_run_event_listener_thread(self):
        """'/ws/vehicles'에 연결하여 '추격 시작(0xFF)' 이벤트를 기다리는 스레드."""
        ws_url = self.base_url 
        while not rospy.is_shutdown():
            try:
                #rospy.loginfo(f"추격 시작 리스너 연결 시도: {ws_url}")
                ws = websocket.create_connection(ws_url, timeout=10)
                rospy.loginfo("추격 시작 리스너 연결 성공.")
                while not rospy.is_shutdown():
                    data = ws.recv()
                    if len(data) == 21 and data[0] == 0xFF:
                        rospy.loginfo(f"[Run Event] 수신: {data.hex(' ')}")
                        _, runner_id = struct.unpack("<BI", data[:5])
                        if not self.chase_active:
                            self.runner_id = runner_id
                            self.chase_active = True
                            self.chase_status_pub.publish("START")
                            rospy.loginfo(f"===== 도주 시작 이벤트 수신! 도주차량 ID: {self.runner_id} =====")
            except Exception as e:
                #rospy.logwarn(f"[Run Event WS] 에러: {e}. 5초 후 재연결...")
                rospy.sleep(5)

    def _ws_catch_broadcast_listener_thread(self):
        """'/ws/events'에 연결하여 '검거 성공(0xFC)' 방송을 기다리는 스레드."""
        ws_url = self.base_url.replace("/vehicles", "/events")
        while not rospy.is_shutdown():
            try:
                #rospy.loginfo(f"검거 방송 리스너 연결 시도: {ws_url}")
                ws = websocket.create_connection(ws_url, timeout=10)
                rospy.loginfo("검거 방송 리스너 연결 성공.")
                while not rospy.is_shutdown():
                    data = ws.recv()
                    if len(data) == 25 and data[0] == 0xFC:
                        #rospy.loginfo(f"[Catch Broadcast] 수신: {data.hex(' ')}")
                        _, catcher_id, runner_id = struct.unpack("<BII", data[:9])
                        rospy.loginfo(f"===== 검거 성공 방송 수신! Catcher: {catcher_id}, Runner: {runner_id} =====")
                        if self.chase_active:
                            self._stop_pursuit() # 검거 방송을 수신하면 추격 중지
            except Exception as e:
                rospy.logwarn(f"[Catch Broadcast WS] 에러: {e}. 5초 후 재연결...")
                rospy.sleep(5)

    def _stop_pursuit(self):
        """추격을 중지하고 ROS 토픽으로 STOP 메시지를 발행합니다."""
        if not self.chase_active: return
        rospy.loginfo("추격 중지 명령 발행.")
        self.chase_active = False
        self.chase_status_pub.publish("STOP")
        stop_cmd = CtrlCmd(longlCmdType=1, accel=0.0, brake=1.0, steering=0.0)
        self.ctrl_pub.publish(stop_cmd)

if __name__ == "__main__":
    try:
        ChaseManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
