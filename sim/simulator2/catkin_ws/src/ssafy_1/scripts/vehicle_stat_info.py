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
BASE_URL_DEFAULT = "ws://52.78.193.190:8080/ws/vehicles"
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
ENV_PATH = "/home/ubuntu/S13P21A507/sim/simulator2/catkin_ws/.env"
load_dotenv(ENV_PATH)
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError(f"HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일({ENV_PATH})을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode("utf-8")


def _hmac16(data: bytes) -> bytes:
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]


# =============================
# 패킷 빌더
# =============================
def build_status_packet(message_type: int, vehicle_id: int,
                        collision_count: int, status_enum: int,
                        fuel: int) -> bytes:
    header8 = struct.pack("<BIBBB", message_type, vehicle_id, fuel,
                          collision_count, status_enum)
    return header8 + _hmac16(header8)


def build_collision_packet(message_type: int, vehicle_id: int,
                           collision_count: int) -> bytes:
    head6 = struct.pack("<BIB", message_type, vehicle_id, collision_count)
    return head6 + _hmac16(head6)


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

        self.send_interval = float(rospy.get_param("~send_interval", SEND_INTERVAL_SEC_DEFAULT))
        self.fuel_check_interval = float(rospy.get_param("~fuel_check_interval", FUEL_CHECK_INTERVAL_SEC_DEFAULT))

        self.half_th = int(rospy.get_param("~half_destroyed_threshold", HALF_DESTROYED_THRESHOLD_DEFAULT))
        self.complete_th = int(rospy.get_param("~complete_destroyed_threshold", COMPLETE_DESTROYED_THRESHOLD_DEFAULT))

        self.force_send_every = float(rospy.get_param("~force_send_every", 0.0))
        self.vehicle_id_override = int(rospy.get_param("~vehicle_id_override", 0))
        self.collision_clear_hold = float(rospy.get_param("~collision_clear_hold", 0.2))

        # 상태 변수
        self.latest_status = None
        self.vehicle_id = 0
        self.fuel = 100
        self.collision_count = 0
        self.in_collision = False
        self._prev_collision_raw = False

        # 전송 상태
        self._last_sent = None
        self._last_sent_ts = 0.0

        # 최근 충돌 신호 시각(디바운스)
        self._collision_last_seen_ts = 0.0

        # Subscribers
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_ego)
        rospy.Subscriber(self.collision_topic, CollisionData, self.on_collision)

        # Timers
        rospy.Timer(rospy.Duration(self.send_interval), self.timer_maybe_send_status)
        rospy.Timer(rospy.Duration(self.fuel_check_interval), self.timer_check_fuel)

        rospy.loginfo(f"[StatusSender] Ready. URL={self.base_url}")

    # ------------- 콜백들 -------------
    def on_ego(self, msg: EgoVehicleStatus):
        self.latest_status = msg
        if self.vehicle_id_override != 0:
            self.vehicle_id = self.vehicle_id_override
        else:
            self.vehicle_id = msg.unique_id if getattr(msg, "unique_id", 0) != 0 else self.vehicle_id

    def on_collision(self, msg: CollisionData):
        cnt_a = len(getattr(msg, "collision_object", []) or [])
        cnt_b = len(getattr(msg, "collision_objecta", []) or [])
        has_collision_raw = (cnt_a + cnt_b) > 0

        # --- 카운트는 RAW 전이로만 ---
        if has_collision_raw and not self._prev_collision_raw:
            self.collision_count = min(self.collision_count + 1, 255)
            rospy.loginfo(f"[StatusSender] Collision RISING edge -> count={self.collision_count}")
            vehicle_id = (self.vehicle_id_override or self.vehicle_id) & 0xFFFFFFFF
            evt_pkt = build_collision_packet(0x13, vehicle_id, int(self.collision_count) & 0xFF)
            self._send_ws_binary(self.base_url, evt_pkt, tag="collision")

        self._prev_collision_raw = has_collision_raw

        # --- 표시 상태는 디바운스 적용 ---
        now_ts = time()
        if has_collision_raw:
            self._collision_last_seen_ts = now_ts
        hold = max(self.collision_clear_hold, 0.0)
        self.in_collision = has_collision_raw or ((hold > 0.0) and ((now_ts - self._collision_last_seen_ts) < hold))

    def timer_check_fuel(self, _evt):
        if self.fuel > 0:
            self.fuel = max(self.fuel - FUEL_DEC_PER_CHECK, 0)

    def timer_maybe_send_status(self, _evt):
        if not self.latest_status:
            return

        vehicle_id = (self.vehicle_id_override or self.vehicle_id) & 0xFFFFFFFF
        status_enum = to_status_enum(self.collision_count, self.half_th, self.complete_th)
        current = (int(self.fuel) & 0xFF,
                   int(self.collision_count) & 0xFF,
                   int(status_enum) & 0xFF)
        now_ts = time()
        must_force = (self.force_send_every > 0.0) and ((now_ts - self._last_sent_ts) >= self.force_send_every)

        if must_force or (self._last_sent is None or current != self._last_sent):
            pkt = build_status_packet(0x12, vehicle_id, current[1], current[2], current[0])
            self._send_ws_binary(self.base_url, pkt, tag="status")
            self._last_sent = current
            self._last_sent_ts = now_ts
            rospy.loginfo(f"[StatusSender] Sent status: id={vehicle_id}, collisions={current[1]}, "
                          f"status={current[2]}, fuel={current[0]} (24B)")

    # ------------- 전송 -------------
    def _send_ws_binary(self, url: str, pkt: bytes, tag=""):
        try:
            ws = websocket.create_connection(url, timeout=3)
            ws.send_binary(pkt)
            ws.close()
        except Exception as e:
            rospy.logwarn(f"[WS] send failed ({tag}): {e}")


# =============================
# 엔트리포인트
# =============================
if __name__ == "__main__":
    rospy.init_node("vehicle_status_sender", anonymous=False)
    VehicleStatusSender()
    rospy.spin()

