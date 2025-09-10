#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import struct
import math
import websocket
import rospy
from morai_msgs.msg import EgoVehicleStatus, VehicleCollisionData

# -----------------------------
# 설정 (ROS 파라미터로도 오버라이드 가능)
# -----------------------------
BASE_URL_DEFAULT = "ws://52.78.193.190:8080/ws/vehicles"
EGO_TOPIC_DEFAULT = "/Ego_topic"
COLLISION_TOPIC_DEFAULT = "/CollisionData"

SEND_INTERVAL_SEC_DEFAULT = 1.0       # 상태 전송 주기
FUEL_CHECK_INTERVAL_SEC_DEFAULT = 5.0 # 연료 감소 판단 주기
MOVE_EPS_DEFAULT = 0.05               # 이동 판정 임계(미터)
FUEL_DEC_PER_CHECK = 1

HALF_DESTROYED_THRESHOLD_DEFAULT = 1  # 1회 이상 → 반파
COMPLETE_DESTROYED_THRESHOLD_DEFAULT = 2  # 2회 이상 → 전파

# -----------------------------
# 패킷 빌더 (HMAC 항상 0으로 채움)
# -----------------------------
def build_status_packet(message_type: int,
                        vehicle_id: int,
                        collision_count: int,
                        status_enum: int,
                        fuel: int) -> bytes:
    """
    레이아웃
    [0]    uint8   : message_type (0x12)
    [1-4]  uint32  : vehicle_id (LE)
    [5]    uint8   : collision_count
    [6]    uint8   : status_enum (0: NORMAL, 1: HALF_DESTROYED, 2: COMPLETE_DESTROYED)
    [7]    uint8   : fuel (0-100)
    [8-23] 16 bytes: HMAC(앞 8바이트에 대한 인증 코드) -> 여기서는 항상 0으로 채움
    총 24 bytes
    """
    header8 = struct.pack("<BIBBB", message_type, vehicle_id, collision_count, status_enum, fuel)
    digest = b"\x00" * 16  # HMAC 미사용 → 0 패딩
    packet = header8 + digest
    assert len(packet) == 24
    return packet

# -----------------------------
# 유틸
# -----------------------------
def dist_xy(x1, y1, x2, y2):
    dx, dy = (x2 - x1), (y2 - y1)
    return math.hypot(dx, dy)

def to_status_enum(collision_count: int, half_th: int, complete_th: int) -> int:
    if collision_count >= complete_th:
        return 2  # COMPLETE_DESTROYED
    if collision_count >= half_th:
        return 1  # HALF_DESTROYED
    return 0      # NORMAL

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

        # 상태 변수
        self.latest_status = None        # EgoVehicleStatus 최신값
        self.vehicle_id = 0
        self.fuel = 90                  # 0~100
        self.collision_count = 0
        self.in_collision = False        # 현재 충돌 지속 상태

        # 이동 판정용
        self.last_pos_for_fuel = None    # (x, y)
        self.pos_at_last_check = None    # 직전 체크 시점 좌표

        # Subscribers
        rospy.Subscriber(self.ego_topic, EgoVehicleStatus, self.on_ego)
        rospy.Subscriber(self.collision_topic, VehicleCollisionData, self.on_collision)

        # Timers
        rospy.Timer(rospy.Duration(self.send_interval), self.timer_send_status)
        rospy.Timer(rospy.Duration(self.fuel_check_interval), self.timer_check_fuel)

        rospy.loginfo(f"[StatusSender] Ready. send_interval={self.send_interval}s, fuel_check={self.fuel_check_interval}s, move_eps={self.move_eps}m")

    # ------------- 콜백들 -------------
    def on_ego(self, msg: EgoVehicleStatus):
        self.latest_status = msg
        self.vehicle_id = msg.unique_id if hasattr(msg, "unique_id") else self.vehicle_id

    def on_collision(self, msg: VehicleCollisionData):
        # collision_object가 비어있지 않으면 '충돌 중'
        has_collision = (len(msg.collision_object) > 0)
        # 상승 에지(비충돌 -> 충돌)에서만 +1
        if has_collision and not self.in_collision:
            self.collision_count = min(self.collision_count + 1, 255)  # uint8 범위
        # 상태 갱신
        self.in_collision = has_collision

    def timer_check_fuel(self, _evt):
        """5초마다 이동 여부를 확인해 연료 감소"""
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
        # 다음 비교를 위해 현재 좌표 저장
        self.pos_at_last_check = (x, y)

    def timer_send_status(self, _evt):
        """주기적으로 상태 패킷 전송"""
        if not self.latest_status:
            return

        # 좌표는 여기선 패킷에 포함되지 않지만, fuel 감소용 기준좌표 갱신은 해 둔다(최초 세팅용)
        if self.last_pos_for_fuel is None:
            self.last_pos_for_fuel = (self.latest_status.position.x, self.latest_status.position.y)

        vehicle_id = int(self.vehicle_id) & 0xFFFFFFFF
        status_enum = to_status_enum(self.collision_count, self.half_th, self.complete_th)

        pkt = build_status_packet(
            message_type=0x12,
            vehicle_id=vehicle_id,
            collision_count=int(self.collision_count) & 0xFF,
            status_enum=int(status_enum) & 0xFF,
            fuel=int(self.fuel) & 0xFF
        )

        url = f"{self.base_url}/{vehicle_id}/status"
        self._send_ws_binary(url, pkt)

        rospy.loginfo(f"[StatusSender] Sent status: id={vehicle_id}, collisions={self.collision_count}, status={status_enum}, fuel={self.fuel}")

    # ------------- 전송 -------------
    def _send_ws_binary(self, url: str, pkt: bytes):
        try:
            ws = websocket.create_connection(url, timeout=2)
            ws.send_binary(pkt)
            ws.close()
        except Exception as e:
            rospy.logwarn(f"[StatusSender] WebSocket send failed: {e}")

# -----------------------------
# 엔트리포인트
# -----------------------------
if __name__ == "__main__":
    rospy.init_node("vehicle_status_sender", anonymous=False)
    VehicleStatusSender()
    rospy.spin()
