#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import struct
import math
import hmac
import hashlib
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus, VehicleCollisionData
from dotenv import load_dotenv
from time import time

# -----------------------------
# 설정 (ROS 파라미터로도 오버라이드 가능)
# -----------------------------
BASE_URL_DEFAULT = "ws://52.78.193.190:8080/ws/vehicles"
EGO_TOPIC_DEFAULT = "/Ego_topic"
COLLISION_TOPIC_DEFAULT = "/VehicleCollisionData"

SEND_INTERVAL_SEC_DEFAULT = 1.0        # 상태 변화 감지 주기(변화 없으면 안보냄)
FUEL_CHECK_INTERVAL_SEC_DEFAULT = 5.0  # 연료 감소 판단 주기(이동 시에만)
MOVE_EPS_DEFAULT = 0.05                # 이동 판정 임계(미터)
FUEL_DEC_PER_CHECK = 1

HALF_DESTROYED_THRESHOLD_DEFAULT = 1
COMPLETE_DESTROYED_THRESHOLD_DEFAULT = 2

# -----------------------------
# HMAC KEY 로드 (.env 절대경로에서)
# -----------------------------
ENV_PATH = "/home/ubuntu/S13P21A507/sim/simulator2/catkin_ws/.env"
load_dotenv(ENV_PATH)
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError(f"HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일({ENV_PATH})을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode("utf-8")

def _hmac16(data: bytes) -> bytes:
    """HMAC-SHA256(data) → 앞 16바이트만"""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

# -----------------------------
# 패킷 빌더 (고정 길이)
# -----------------------------
def build_status_packet(message_type: int,
                        vehicle_id: int,
                        collision_count: int,
                        status_enum: int,
                        fuel: int) -> bytes:
    """
    상태 패킷 (총 24B)
    [0]  u8  : message_type (0x12)
    [1-4] u32 LE : vehicle_id
    [5]  u8  : fuel (0-100)
    [6]  u8  : collision_count
    [7]  u8  : status_enum (0 NORMAL,1 HALF,2 COMPLETE)
    [8-23] 16B : HMAC([0:8])
    """
    header8 = struct.pack("<BIBBB", message_type, vehicle_id, fuel, collision_count, status_enum)
    return header8 + _hmac16(header8)  # => 8 + 16 = 24B

def build_collision_packet(message_type: int,
                           vehicle_id: int,
                           collision_count: int) -> bytes:
    """
    충돌 이벤트 패킷 (총 22B)
    [0]  u8  : message_type (0x13)
    [1-4] u32 LE : vehicle_id
    [5]  u8  : collision_count
    [6-21] 16B : HMAC([0:6])
    """
    head6 = struct.pack("<BIB", message_type, vehicle_id, collision_count)
    return head6 + _hmac16(head6)  # => 6 + 16 = 22B

# -----------------------------
# 유틸
# -----------------------------
def dist_xy(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)

NORMAL, HALF, COMPLETE = 0, 1, 2
def to_status_enum(collision_count: int, half_th: int, complete_th: int) -> int:
    if collision_count >= complete_th: return COMPLETE
    if collision_count >= half_th:     return HALF
    return NORMAL

def bhex(b: bytes, limit=48) -> str:
    s = b.hex()
    return s if len(s) <= limit else s[:limit] + "...(" + str(len(b)) + "B)"

# -----------------------------
# 메인 클래스
# -----------------------------
class VehicleStatusSender:
    def __init__(self):
        # ROS params
        self.base_url = rospy.get_param("~base_url", BASE_URL_DEFAULT)
        self.ego_topic = rospy.get_param("~ego_topic", EGO_TOPIC_DEFAULT)
        self.collision_topic = rospy.get_param("~collision_topic", COLLISION_TOPIC_DEFAULT)

        self.send_interval = float(rospy.get_param("~send_interval", SEND_INTERVAL_SEC_DEFAULT))
        self.fuel_check_interval = float(rospy.get_param("~fuel_check_interval", FUEL_CHECK_INTERVAL_SEC_DEFAULT))
        self.move_eps = float(rospy.get_param("~move_eps", MOVE_EPS_DEFAULT))

        self.half_th = int(rospy.get_param("~half_destroyed_threshold", HALF_DESTROYED_THRESHOLD_DEFAULT))
        self.complete_th = int(rospy.get_param("~complete_destroyed_threshold", COMPLETE_DESTROYED_THRESHOLD_DEFAULT))

        # ---- 편의 옵션(패킷 포맷에는 영향 X) ----
        self.force_send_every = float(rospy.get_param("~force_send_every", 0.0))  # 0이면 비활성
        self.vehicle_id_override = int(rospy.get_param("~vehicle_id_override", 0)) # 0이면 미사용
        self.collision_clear_hold = float(rospy.get_param("~collision_clear_hold", 0.0)) # 디바운스(초), 0이면 비활성
        self._collision_last_seen_ts = 0.0

        # 상태 변수
        self.latest_status = None
        self.vehicle_id = 0
        self.fuel = 100
        self.collision_count = 0
        self.in_collision = False

        # 전송 상태
        self._last_sent = None  # (fuel, collision_count, status_enum)
        self._last_sent_ts = 0.0

        # 이동 판정용
        self.pos_at_last_check = None

        # Subscribers
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_ego)
        rospy.Subscriber(self.collision_topic, VehicleCollisionData, self.on_collision)

        # Timers
        rospy.Timer(rospy.Duration(self.send_interval), self.timer_maybe_send_status)
        rospy.Timer(rospy.Duration(self.fuel_check_interval), self.timer_check_fuel)

        rospy.loginfo(f"[StatusSender] Ready. send_interval={self.send_interval}s, fuel_check={self.fuel_check_interval}s, move_eps={self.move_eps}m (Fixed packet sizes: status=24B, collision=22B)")

    # ------------- 콜백들 -------------
    def on_ego(self, msg: EgoVehicleStatus):
        self.latest_status = msg
        if self.vehicle_id_override != 0:
            self.vehicle_id = self.vehicle_id_override
        else:
            self.vehicle_id = msg.unique_id if hasattr(msg, "unique_id") and msg.unique_id != 0 else self.vehicle_id

    def on_collision(self, msg: VehicleCollisionData):
        """
        VehicleCollisionData:
        collisions: [] 또는 [{ crashed_vehicles: [...] }, ...]
        """
        # 기본 수신 로그
        try:
            seq = getattr(msg.header, "seq", -1)
        except Exception:
            seq = -1

        # raw 충돌 여부
        try:
            has_collision_raw = hasattr(msg, "collisions") and (msg.collisions is not None) and (len(msg.collisions) > 0)
        except Exception:
            has_collision_raw = False

        # 디바운스 적용
        now_ts = time()
        if has_collision_raw:
            self._collision_last_seen_ts = now_ts
            has_collision = True
        else:
            if self.collision_clear_hold > 0 and (now_ts - self._collision_last_seen_ts) < self.collision_clear_hold:
                has_collision = True
            else:
                has_collision = False

        # 상세 디버그: 충돌 페어/차량 id/이름
        #if has_collision_raw:
        #    try:
        #        pairs = len(msg.collisions)
        #        first_pair = msg.collisions[0]
        #        vlist = getattr(first_pair, "crashed_vehicles", [])
        #        ids = []
        #        names = []
        #        for v in vlist:
        #            vid = getattr(v, "unique_id", -1)
        #            vname = getattr(v, "name", "")
        #            ids.append(vid)
        #            names.append(vname)
        #         rospy.loginfo(f"[Collision RX] seq={seq} pairs={pairs} first_pair_vehicles={len(vlist)} ids={ids} names={names}")
        #    except Exception as e:
        #         rospy.logwarn(f"[Collision RX] seq={seq} parse warn: {e}")
        #else:
            #rospy.loginfo(f"[Collision RX] seq={seq} collisions=[]")

        # 상승 에지에서만 카운트 증가
        if has_collision and not self.in_collision:
            self.collision_count = min(self.collision_count + 1, 255)
            rospy.loginfo(f"[StatusSender] Collision RISING edge -> count={self.collision_count}")

            # 충돌 이벤트 패킷(22B) 전송
            vehicle_id = (self.vehicle_id_override or self.vehicle_id) & 0xFFFFFFFF
            evt_pkt = build_collision_packet(0x13, vehicle_id, int(self.collision_count) & 0xFF)
            evt_url = f"{self.base_url}/{vehicle_id}/collision"
            self._send_ws_binary(evt_url, evt_pkt, tag="collision")

        # 하강 에지 로그(선택)
        if (not has_collision) and self.in_collision:
            rospy.loginfo(f"[StatusSender] Collision FALLING edge")

        # 상태 갱신
        self.in_collision = has_collision

    def timer_check_fuel(self, _evt):
        """연료 감소 판단(이동 시에만 감소)"""
        if not self.latest_status:
            return
        x = self.latest_status.position.x
        y = self.latest_status.position.y

        if self.pos_at_last_check is None:
            self.pos_at_last_check = (x, y)
            return

        moved = dist_xy(self.pos_at_last_check[0], self.pos_at_last_check[1], x, y) > self.move_eps
        if moved and self.fuel > 0:
            self.fuel = max(self.fuel - FUEL_DEC_PER_CHECK, 0)
        self.pos_at_last_check = (x, y)

    def timer_maybe_send_status(self, _evt):
        if not self.latest_status:
            return

        vehicle_id = (self.vehicle_id_override or self.vehicle_id) & 0xFFFFFFFF
        status_enum = to_status_enum(self.collision_count, self.half_th, self.complete_th)

        current = (int(self.fuel) & 0xFF, int(self.collision_count) & 0xFF, int(status_enum) & 0xFF)
        now_ts = time()
        must_force = (self.force_send_every > 0.0) and ((now_ts - self._last_sent_ts) >= self.force_send_every)

        if must_force or (self._last_sent is None or current != self._last_sent):
            pkt = build_status_packet(
                message_type=0x12,
                vehicle_id=vehicle_id,
                collision_count=current[1],
                status_enum=current[2],
                fuel=current[0]
            )
            url = f"{self.base_url}/{vehicle_id}/status"
            self._send_ws_binary(url, pkt, tag="status")
            self._last_sent = current
            self._last_sent_ts = now_ts
            rospy.loginfo(f"[StatusSender] Sent status: id={vehicle_id}, collisions={current[1]}, status={current[2]}, fuel={current[0]} (len=24B)")

    # ------------- 전송 -------------
    def _send_ws_binary(self, url: str, pkt: bytes, tag=""):
        try:
            ws = websocket.create_connection(url, timeout=3)
            ws.send_binary(pkt)
            #rospy.loginfo(f"[WS TX] {tag} -> {url} | {len(pkt)}B | head_hex={bhex(pkt[:12])}")
            # 응답 수신 시도(텍스트/바이너리)
            ws.settimeout(1.0)
            #try:
            #    resp = ws.recv()
            #    if isinstance(resp, (bytes, bytearray)):
            #        rospy.loginfo(f"[WS RX] {tag} <- {len(resp)}B bin | head_hex={bhex(resp[:12])}")
            #    else:
            #        rospy.loginfo(f"[WS RX] {tag} <- text: {str(resp)[:120]}")
            #except Exception:
            #    pass
            ws.close()
        except Exception as e:
            rospy.logwarn(f"[WS] send failed ({tag}): {e}")

# -----------------------------
# 엔트리포인트
# -----------------------------
if __name__ == "__main__":
    rospy.init_node("vehicle_status_sender", anonymous=False)
    VehicleStatusSender()
    rospy.spin()
