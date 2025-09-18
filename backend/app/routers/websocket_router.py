# app/routers/websocket_router.py

import struct
from typing import List

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

# --- 프로젝트 내부 모듈 ---
from sqlalchemy import select
from sqlalchemy.orm import selectinload
from app.models import models
from app.common.ws_codes import MessageType
from app.db import AsyncSessionMaker, get_all_vehicles
from app.models.enums import VehicleTypeEnum, PoliceCarStatusEnum, EventStatus
from app.services import websocket_service
from app.services.websocket_service import _calculate_hmac

# --- 연결 관리자 클래스 (변경 없음) ---
class ConnectionManager:
    """
    ROS-서버-Front 양방향, 동시 다중 통신을 지원하는 웹소켓 매니저
    """
    def __init__(self):
        self.ros_connections: List[WebSocket] = []
        self.front_connections: List[WebSocket] = []

    # --- ROS 클라이언트 관리 ---
    async def connect_ros(self, websocket: WebSocket):
        await websocket.accept()
        self.ros_connections.append(websocket)
        print(f"ROS 클라이언트 연결. 총 ROS 클라이언트: {len(self.ros_connections)}")

    def disconnect_ros(self, websocket: WebSocket):
        if websocket in self.ros_connections:
            self.ros_connections.remove(websocket)
            print(f"ROS 클라이언트 연결 해제. 총 ROS 클라이언트: {len(self.ros_connections)}")

    # --- 프론트엔드 클라이언트 관리 ---
    async def connect_front(self, websocket: WebSocket):
        await websocket.accept()
        self.front_connections.append(websocket)
        print(f"프론트엔드 클라이언트 연결. 총 프론트엔드: {len(self.front_connections)}")

    def disconnect_front(self, websocket: WebSocket):
        if websocket in self.front_connections:
            self.front_connections.remove(websocket)
            print(f"프론트엔드 클라이언트 연결 해제. 총 프론트엔드: {len(self.front_connections)}")

    # --- 브로드캐스트 메서드 ---
    async def broadcast_to_front(self, message: bytes):
        """모든 프론트엔드 클라이언트에게 메시지를 전송합니다."""
        for connection in list(self.front_connections):
            try:
                await connection.send_bytes(message)
            except (WebSocketDisconnect, RuntimeError):
                self.disconnect_front(connection)

    # 💡 변경된 메서드: '모든' ROS 클라이언트에게 브로드캐스트
    async def broadcast_to_all_ros(self, message: bytes):
        """
        '모든' ROS 클라이언트(메시지를 보낸 클라이언트 포함)에게 메시지를 브로드캐스트합니다.
        """
        # 💡 'sender'를 구별할 필요 없이 그냥 전부 전송
        for connection in self.ros_connections:
            try:
                await connection.send_bytes(message)
            except (WebSocketDisconnect, RuntimeError):
                self.disconnect_ros(connection)

# --- 전역 매니저 객체 생성 ---
vehicle_manager = ConnectionManager()
event_manager = ConnectionManager()
# 서비스 계층에 두 매니저를 모두 전달하기 위한 딕셔너리
managers = {"vehicle": vehicle_manager, "event": event_manager}
router = APIRouter(tags=["WebSockets"])


async def send_initial_vehicle_data(websocket: WebSocket):
    # 이 함수는 프론트엔드 UI 초기화를 위한 것이므로 id를 사용합니다.
    print("새 클라이언트에게 기존 차량 데이터를 전송합니다.")
    async with AsyncSessionMaker() as db_session:
        all_vehicles = await get_all_vehicles(db_session)
        positions_data = []
        for vehicle in all_vehicles:
            if vehicle.locations:
                for location in vehicle.locations:
                    positions_data.append(struct.pack('<Iff', vehicle.id, location.position_x, location.position_y))
        if positions_data:
            header = struct.pack('<BI', MessageType.POSITION_BROADCAST_2D, len(positions_data))
            packed_positions = b"".join(positions_data)
            full_message = header + packed_positions
            await websocket.send_bytes(full_message + _calculate_hmac(full_message))
        for vehicle in all_vehicles:
            car_name_padded = vehicle.car_name.encode('utf-8').ljust(10, b'\x00')
            vehicle_type_int = 0 if vehicle.vehicle_type == VehicleTypeEnum.POLICE else 1
            reg_header = struct.pack('<BIIB10s', MessageType.EVENT_VEHICLE_REGISTERED, vehicle.id, vehicle.vehicle_id, vehicle_type_int, car_name_padded)
            await websocket.send_bytes(reg_header + _calculate_hmac(reg_header))
            if vehicle.vehicle_type == VehicleTypeEnum.POLICE and vehicle.police_car:
                status_map = {p_status: i for i, p_status in enumerate(PoliceCarStatusEnum)}
                status_int = status_map.get(vehicle.police_car.status, 0)
                status_header = struct.pack('<BIBBB', MessageType.STATE_UPDATE, vehicle.id, vehicle.police_car.collision_count, status_int, vehicle.police_car.fuel)
                await websocket.send_bytes(status_header + _calculate_hmac(status_header))
    print(f"기존 차량 데이터(위치, 상태 포함) 전송 완료: 총 {len(all_vehicles)}대")

async def send_initial_event_data(websocket: WebSocket):
    """
    새로운 프론트엔드 클라이언트에게 DB의 모든 과거 이벤트 데이터를 전송합니다.
    """
    print("새 클라이언트에게 기존 이벤트 데이터를 전송합니다.")
    async with AsyncSessionMaker() as db_session:
        # 1. DB에서 모든 이벤트를 조회합니다. (N+1 문제를 피하기 위해 runner와 catcher 정보를 함께 로딩)
        statement = select(models.Event).options(
            selectinload(models.Event.runner),
            selectinload(models.Event.catcher)
        ).order_by(models.Event.created_at) # 시간 순으로 보내기 위해 정렬
        
        result = await db_session.execute(statement)
        all_events = result.scalars().all()

        # 2. 각 이벤트를 순회하며 상태에 맞는 바이너리 패킷을 생성하여 전송합니다.
        for event in all_events:
            header_packet = None
            
            # --- 추적 시작 이벤트 (RUN) ---
            if event.status == EventStatus.RUN and event.runner:
                # 프론트엔드용 추적 시작 이벤트 타입은 0xF0 입니다.
                # runner.id는 DB의 PK id를 사용합니다.
                header_packet = struct.pack('<BI', MessageType.EVENT_TRACE_START, event.runner.id)
            # --- 검거 성공 이벤트 (CATCH) ---
            elif event.status == EventStatus.CATCH and event.catcher and event.runner:
                header_packet = struct.pack('<BII', MessageType.EVENT_CATCH, event.catcher.id, event.runner.id)
            # --- 검거 실패 이벤트 (FAILED) ---
            elif event.status == EventStatus.FAILED and event.catcher and event.runner:
                header_packet = struct.pack('<BII', MessageType.EVENT_CATCH_FAILED, event.catcher.id, event.runner.id)
            
            # 3. 생성된 패킷이 있으면 HMAC을 추가하여 전송합니다.
            if header_packet:
                await websocket.send_bytes(header_packet + _calculate_hmac(header_packet))
                
    print(f"기존 이벤트 데이터 전송 완료: 총 {len(all_events)}건")

@router.websocket("/ws/front/vehicles")
async def websocket_front_vehicles(websocket: WebSocket):
    """프론트엔드 차량 정보용 웹소켓 (초기 데이터 수신 후, 실시간 업데이트 대기)"""
    await vehicle_manager.connect_front(websocket)
    try:
        # 1. 연결 직후, UI 초기 구성을 위해 DB의 모든 차량 데이터를 전송
        await send_initial_vehicle_data(websocket)

        # 2. 프론트엔드는 메시지를 보내지 않으므로, 연결을 유지하며 수신만 대기
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        print("프론트엔드 차량 클라이언트 연결 해제")
    finally:
        vehicle_manager.disconnect_front(websocket)


@router.websocket("/ws/front/events")
async def websocket_front_events(websocket: WebSocket):
    """프론트엔드 이벤트 정보용 웹소켓 (초기 데이터 수신 후, 실시간 업데이트 대기)"""
    await event_manager.connect_front(websocket)
    try:
        # 1. 연결 직후, UI 초기 구성을 위해 DB의 모든 이벤트 데이터를 전송
        await send_initial_event_data(websocket)

        # 2. 프론트엔드는 메시지를 보내지 않으므로, 연결을 유지하며 수신만 대기
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        print("프론트엔드 이벤트 클라이언트 연결 해제")
    finally:
        event_manager.disconnect_front(websocket)


@router.websocket("/ws/vehicles")
async def websocket_ros_vehicles(websocket: WebSocket):
    """ROS 차량 데이터(등록, 위치, 상태) 수신 및 처리용 엔드포인트"""
    await vehicle_manager.connect_ros(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            if not data: continue

            # 💡 핵심 변경점: 데이터 길이 대신 첫 바이트(메시지 타입)로 분기
            message_type = data[0]

            ros_response, front_event, ros_broadcast_event = None, None, None

            if message_type == MessageType.REGISTER_REQUEST: # 0xA0
                ros_response, front_event, ros_broadcast_event = await websocket_service.handle_vehicle_registration(data)
            
            elif message_type == MessageType.POSITION_BROADCAST: # 0x13
                ros_response, front_event, ros_broadcast_event = await websocket_service.handle_location_update(data)
            
            elif message_type == MessageType.STATUS_UPDATE_REQUEST: # 0x12
                ros_response, front_event, ros_broadcast_event = await websocket_service.handle_vehicle_status_update(data)
            
            else:
                print(f"[/ws/vehicles] 정의되지 않은 메시지 타입 수신: {hex(message_type)}")

            # 1. ROS 송신 측에 대한 직접 응답 (ACK/NACK 등)
            if ros_response:
                await websocket.send_bytes(ros_response)

            # 2. 프론트엔드 클라이언트 전체에 브로드캐스트
            if front_event:
                await vehicle_manager.broadcast_to_front(front_event)
            
            # 3. 모든 ROS 클라이언트 (송신자 포함)에 이벤트 브로드캐스트
            if ros_broadcast_event:
                await vehicle_manager.broadcast_to_all_ros(ros_broadcast_event)

    except WebSocketDisconnect:
        print("ROS 차량 클라이언트 연결 해제")
    finally:
        vehicle_manager.disconnect_ros(websocket)


@router.websocket("/ws/events")
async def websocket_ros_events(websocket: WebSocket):
    """ROS 게임 이벤트(검거 등) 수신 및 처리용 엔드포인트"""
    await event_manager.connect_ros(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            if not data: continue
            
            message_type = data[0]
            ros_response, front_broadcast, ros_broadcast = None, None, None

            if message_type in [MessageType.EVENT_CATCH, MessageType.EVENT_CATCH_FAILED]:
                ros_response, front_broadcast, ros_broadcast = await websocket_service.handle_incoming_event(data)
            else:
                 print(f"[/ws/events] 정의되지 않은 메시지 타입 수신: {hex(message_type)}")

            if ros_response:
                await websocket.send_bytes(ros_response)
            if front_broadcast:
                await event_manager.broadcast_to_front(front_broadcast)
            if ros_broadcast:
                await event_manager.broadcast_to_all_ros(ros_broadcast)

    except WebSocketDisconnect:
        print("ROS 이벤트 클라이언트 연결 해제")
    finally:
        event_manager.disconnect_ros(websocket)