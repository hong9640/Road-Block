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
from time import time

# =============================
# 설정
# =============================
BASE_URL_DEFAULT = "wss://j13a507.p.ssafy.io/ws/vehicles"
EGO_TOPIC_DEFAULT = "/Ego_topic"
COLLISION_TOPIC_DEFAULT = "/CollisionData"
SEND_INTERVAL_SEC_DEFAULT = 1.0
FUEL_CHECK_INTERVAL_SEC_DEFAULT = 5.0
FUEL_DEC_PER_CHECK = 1

HALF_DESTROYED_THRESHOLD_DEFAULT = 1
COMPLETE_DESTROYED_THRESHOLD_DEFAULT = 2

# =============================
# HMAC KEY 로드
# =============================
ENV_PATH = "/home/ubuntu/S13P21A507/sim/simulator1/catkin_ws/.env"
load_dotenv(ENV_PATH)
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError(f"HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일({ENV_PATH})을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode("utf-8")

CATCHER_NAME_TO_ID_MAP = {
    "2016_Hyundai_Genesis_DH": 0,
    "2019_Toyota_Prius": 3,
    "2019_hyundai_genesis_g90": 4,
}

def _hmac16(data: bytes) -> bytes:
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]


# =============================
# 패킷 빌더
# =============================
def build_status_packet(message_type: int, vehicle_id: int,
                        fuel: int, collision_count: int,
                        status_enum: int) -> bytes:
    header8 = struct.pack("<BIBBB", message_type, vehicle_id, fuel,
                          collision_count, status_enum)
    return header8 + _hmac16(header8)


def build_collision_packet(message_type: int, vehicle_id: int,
                           collision_count: int) -> bytes:
    head6 = struct.pack("<BIB", message_type, vehicle_id, collision_count)
    return head6 + _hmac16(head6)

# <<<< 검거 이벤트(0xFE) 패킷 빌더
def build_catch_event_packet(catcher_id: int, runner_id: int) -> bytes:
    """
    [0]     uint8   : message_type (0xFE)
    [1-4]   uint32 LE : catcher_id
    [5-8]   uint32 LE : runner_id
    [9-24]  16 bytes  : hmac
    """
    header9 = struct.pack("<BII", 0xFE, catcher_id, runner_id)
    return header9 + _hmac16(header9)


# =============================
# 유틸
# =============================
NORMAL, HALF, COMPLETE = 0, 1, 2
def to_status_enum(collision_count: int, half_th: int, complete_th: int) -> int:
    if collision_count >= complete_th: return COMPLETE
    if collision_count >= half_th:     return HALF
    return NORMAL


# =============================
# 메인 클래스
# =============================
class VehicleStatusSender:
    def __init__(self):
        self.base_url = rospy.get_param("~base_url", BASE_URL_DEFAULT)
        self.ego_topic = rospy.get_param("~ego_topic", EGO_TOPIC_DEFAULT)
        self.collision_topic = rospy.get_param("~collision_topic", COLLISION_TOPIC_DEFAULT)

        # <<<< 차량 타입을 파라미터로 받아 역할 구분 (0: 경찰, 1: 도주차)
        self.vehicle_type = int(rospy.get_param("~vehicle_type", 0))

        self.send_interval = float(rospy.get_param("~send_interval", SEND_INTERVAL_SEC_DEFAULT))
        self.fuel_check_interval = float(rospy.get_param("~fuel_check_interval", FUEL_CHECK_INTERVAL_SEC_DEFAULT))
        self.vehicle_id_override = int(rospy.get_param("~vehicle_id_override", 0))
        self.half_th = int(rospy.get_param("~half_destroyed_threshold", HALF_DESTROYED_THRESHOLD_DEFAULT))
        self.complete_th = int(rospy.get_param("~complete_destroyed_threshold", COMPLETE_DESTROYED_THRESHOLD_DEFAULT))
        self.force_send_every = float(rospy.get_param("~force_send_every", 0.0))
        self.collision_clear_hold = float(rospy.get_param("~collision_clear_hold", 0.2))

        # 상태 변수
        self.latest_status = None
        self.vehicle_id = 0
        self.fuel = 100
        self.collision_count = 0
        self.in_collision = False
        self._prev_collision_raw = False
        self.catch_event_sent = False # <<<< 검거 이벤트는 한 번만 보내기 위한 플래그

        # 전송 상태
        self._last_sent = None
        self._last_sent_ts = 0.0
        self._collision_last_seen_ts = 0.0

        # Subscribers
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_ego)
        rospy.Subscriber(self.collision_topic, CollisionData, self.on_collision)

        # Timers
        rospy.Timer(rospy.Duration(self.send_interval), self.timer_maybe_send_status)
        rospy.Timer(rospy.Duration(self.fuel_check_interval), self.timer_check_fuel)

        role = "Runner" if self.vehicle_type == 1 else "Police"
        rospy.loginfo(f"[StatusSender] Ready. Role={role}, URL={self.base_url}")

    # ------------- 콜백들 -------------
    def on_ego(self, msg: EgoVehicleStatus):
        self.latest_status = msg
        if self.vehicle_id_override != 0:
            self.vehicle_id = self.vehicle_id_override
        else:
            self.vehicle_id = msg.unique_id

    def on_collision(self, msg: CollisionData):
        cnt_a = len(getattr(msg, "collision_object", []) or [])
        cnt_b = len(getattr(msg, "collision_objecta", []) or [])
        has_collision_raw = (cnt_a + cnt_b) > 0

        # <<<< 충돌 발생 시 차량 타입에 따라 다른 동작 수행
        if has_collision_raw and not self._prev_collision_raw:
            self.collision_count = min(self.collision_count + 1, 255)

            # [분기 1] 만약 이 노드가 "도주 차량" (type 1) 이라면
            if self.vehicle_type == 1 and not self.catch_event_sent:
                if msg.collision_object:
                    collided_object = msg.collision_object[0]
                    collided_name = collided_object.name
                    catcher_id = collided_object.unique_id
                    
                    if collided_name in CATCHER_NAME_TO_ID_MAP:
                        overridden_id = CATCHER_NAME_TO_ID_MAP[collided_name]
                        rospy.loginfo(f"Collision with '{collided_name}'. Overriding catcher_id from {catcher_id} to {overridden_id}.")
                        catcher_id = overridden_id
                    
                    runner_id = self.vehicle_id
                    rospy.loginfo(f"!!! RUNNER CAUGHT by {catcher_id} !!! Sending CATCH_EVENT(0xFE) to /ws/events")

                    # 검거 이벤트 패킷 생성 및 /ws/events 로 전송
                    event_pkt = build_catch_event_packet(int(catcher_id), int(runner_id))
                    event_url = self.base_url.replace("/vehicles", "/events")
                    self._send_ws_binary(event_url, event_pkt, tag="catch_event")
                    self.catch_event_sent = True # 중복 전송 방지
                else:
                    rospy.logwarn("Runner collided but could not identify the catcher.")
            
            # [분기 2] "경찰차" (type 0) 이거나 기본 동작일 경우
            else:
                rospy.loginfo(f"[StatusSender] Collision RISING edge -> count={self.collision_count}")
                vehicle_id = (self.vehicle_id_override or self.vehicle_id) & 0xFFFFFFFF
                # 기존과 동일하게 충돌 카운트(0x13)를 /ws/vehicles 로 전송
                evt_pkt = build_collision_packet(0x13, vehicle_id, int(self.collision_count) & 0xFF)
                self._send_ws_binary(self.base_url, evt_pkt, tag="collision")

        self._prev_collision_raw = has_collision_raw
        now_ts = time()
        if has_collision_raw:
            self._collision_last_seen_ts = now_ts
        hold = max(self.collision_clear_hold, 0.0)
        self.in_collision = has_collision_raw or ((hold > 0.0) and ((now_ts - self._collision_last_seen_ts) < hold))

    def timer_check_fuel(self, _evt):
        if self.fuel > 0:
            self.fuel = max(self.fuel - FUEL_DEC_PER_CHECK, 0)

    def timer_maybe_send_status(self, _evt):
        if not self.latest_status: return

        vehicle_id = (self.vehicle_id_override or self.vehicle_id) & 0xFFFFFFFF
        status_enum = to_status_enum(self.collision_count, self.half_th, self.complete_th)
        current = (int(self.fuel) & 0xFF,
                   int(self.collision_count) & 0xFF,
                   int(status_enum) & 0xFF)
        now_ts = time()
        must_force = (self.force_send_every > 0.0) and ((now_ts - self._last_sent_ts) >= self.force_send_every)

        if must_force or (self._last_sent is None or current != self._last_sent):
            pkt = build_status_packet(
                message_type=0x12,
                vehicle_id=vehicle_id,
                fuel=current[0],
                collision_count=current[1],
                status_enum=current[2]
            )
            self._send_ws_binary(self.base_url, pkt, tag="status")
            self._last_sent = current
            self._last_sent_ts = now_ts
            rospy.loginfo(f"[StatusSender] Sent status: id={vehicle_id}, collisions={current[1]}, "
                          f"status={current[2]}, fuel={current[0]} (24B)")

    def _send_ws_binary(self, url: str, pkt: bytes, tag=""):
        try:
            ws = websocket.create_connection(url, timeout=3)
            ws.send_binary(pkt)
            ws.close()
        except Exception as e:
            rospy.logwarn(f"[WS] send failed ({tag}): {e}")


if __name__ == "__main__":
    rospy.init_node("vehicle_status_sender", anonymous=False)
    VehicleStatusSender()
    rospy.spin()
