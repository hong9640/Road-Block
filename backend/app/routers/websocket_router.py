# app/routers/websocket_router.py
import logging
from logging.handlers import RotatingFileHandler
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

# --- 1. 파일 로거 설정 ---
event_log = logging.getLogger('event_broadcast_logger')
event_log.setLevel(logging.INFO)
handler = RotatingFileHandler('event_broadcasts.log', maxBytes=1024*1024*5, backupCount=5, encoding='utf-8')
formatter = logging.Formatter('%(asctime)s - %(message)s')
handler.setFormatter(formatter)
if not event_log.handlers:
    event_log.addHandler(handler)

# --- 2. 연결 관리자 클래스 ---
class ConnectionManager:
    def __init__(self):
        self.ros_connections: List[WebSocket] = []
        self.front_connections: List[WebSocket] = []

    async def connect_ros(self, websocket: WebSocket):
        await websocket.accept()
        self.ros_connections.append(websocket)
        logging.info(f"ROS 클라이언트 연결. 총 ROS: {len(self.ros_connections)}")

    def disconnect_ros(self, websocket: WebSocket):
        if websocket in self.ros_connections:
            self.ros_connections.remove(websocket)
            logging.info(f"ROS 클라이언트 연결 해제. 총 ROS: {len(self.ros_connections)}")

    async def connect_front(self, websocket: WebSocket):
        await websocket.accept()
        self.front_connections.append(websocket)
        logging.info(f"프론트엔드 클라이언트 연결. 총 프론트엔드: {len(self.front_connections)}")

    def disconnect_front(self, websocket: WebSocket):
        if websocket in self.front_connections:
            self.front_connections.remove(websocket)
            logging.info(f"프론트엔드 클라이언트 연결 해제. 총 프론트엔드: {len(self.front_connections)}")

    async def broadcast_to_front(self, message: bytes):
        for connection in list(self.front_connections):
            try:
                await connection.send_bytes(message)
            except (WebSocketDisconnect, RuntimeError):
                self.disconnect_front(connection)

    async def broadcast_to_all_ros(self, message: bytes):
        for connection in list(self.ros_connections):
            try:
                await connection.send_bytes(message)
            except (WebSocketDisconnect, RuntimeError):
                self.disconnect_ros(connection)

# --- 전역 객체 생성 ---
vehicle_manager = ConnectionManager()
event_manager = ConnectionManager()
router = APIRouter(tags=["WebSockets"])

async def send_initial_vehicle_data(websocket: WebSocket):
    # 이 함수는 프론트엔드 UI 초기화를 위한 것이므로 id를 사용합니다.
    print("새 클라이언트에게 기존 차량 데이터를 전송합니다.")
    async with AsyncSessionMaker() as db_session:
        all_vehicles = await get_all_vehicles(db_session)
        positions_data = []
        for vehicle in all_vehicles:
            if vehicle.locations:
                latest_location = vehicle.locations[-1]
                positions_data.append(
                    struct.pack('<Iff', vehicle.id, latest_location.position_x, latest_location.position_y)
                )
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

# --- 프론트엔드 엔드포인트 ---
@router.websocket("/ws/front/vehicles")
async def websocket_front_vehicles(websocket: WebSocket):
    await vehicle_manager.connect_front(websocket)
    try:
        while True:
            await websocket.receive_text()
    except (WebSocketDisconnect, RuntimeError):
        logging.warning("프론트엔드 차량 클라이언트 연결 종료")
    finally:
        vehicle_manager.disconnect_front(websocket)

@router.websocket("/ws/front/events")
async def websocket_front_events(websocket: WebSocket):
    await event_manager.connect_front(websocket)
    try:
        while True:
            await websocket.receive_text()
    except (WebSocketDisconnect, RuntimeError):
        logging.warning("프론트엔드 이벤트 클라이언트 연결 종료")
    finally:
        event_manager.disconnect_front(websocket)

# --- ROS 엔드포인트 ---
@router.websocket("/ws/vehicles")
async def websocket_ros_vehicles(websocket: WebSocket):
    await vehicle_manager.connect_ros(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            if not data: continue
            message_type = data[0]
            ros_response, front_events, ros_broadcast = None, None, None

            if message_type == MessageType.REGISTER_REQUEST:
                ros_response, front_events, ros_broadcast = await websocket_service.handle_vehicle_registration(data)
                if front_events:
                    if 'front_vehicle_event' in front_events:
                        await vehicle_manager.broadcast_to_front(front_events['front_vehicle_event'])
                    if 'front_game_event' in front_events:
                        event_packet = front_events['front_game_event']
                        event_log.info(f"BROADCAST EVENT: {event_packet.hex()}")
                        await event_manager.broadcast_to_front(event_packet)

            elif message_type == MessageType.POSITION_BROADCAST:
                ros_response, front_events, ros_broadcast = await websocket_service.handle_location_update(data)
                # 차량 위치 정보
                if front_events and 'front_vehicle_event' in front_events:
                    await vehicle_manager.broadcast_to_front(front_events['front_vehicle_event'])
                    # await event_manager.broadcast_to_front(front_events['front_vehicle_event'])
            
            elif message_type == MessageType.STATUS_UPDATE_REQUEST:
                ros_response, front_events, ros_broadcast = await websocket_service.handle_vehicle_status_update(data)
                if front_events and 'front_vehicle_event' in front_events:
                    await vehicle_manager.broadcast_to_front(front_events['front_vehicle_event'])

            if ros_response:
                await websocket.send_bytes(ros_response)
            if ros_broadcast:
                await vehicle_manager.broadcast_to_all_ros(ros_broadcast)

    except (WebSocketDisconnect, RuntimeError) as e:
        logging.warning(f"ROS 차량 클라이언트 연결 종료: {e}")
    finally:
        vehicle_manager.disconnect_ros(websocket)

@router.websocket("/ws/events")
async def websocket_ros_events(websocket: WebSocket):
    await event_manager.connect_ros(websocket)
    try:
        while True:
            data = await websocket.receive_bytes()
            if not data: continue
            message_type = data[0]

            if message_type in [MessageType.EVENT_CATCH, MessageType.EVENT_CATCH_FAILED]:
                ros_response, front_broadcast, ros_broadcast = await websocket_service.handle_incoming_event(data)
                if ros_response:
                    await websocket.send_bytes(ros_response)
                if front_broadcast:
                    event_log.info(f"BROADCAST EVENT: 등록완료")
                    await event_manager.broadcast_to_front(front_broadcast)
                    # await vehicle_manager.broadcast_to_front(front_broadcast)
                if ros_broadcast:
                    await event_manager.broadcast_to_all_ros(ros_broadcast)

    except (WebSocketDisconnect, RuntimeError) as e:
        logging.warning(f"ROS 이벤트 클라이언트 연결 종료: {e}")
    finally:
        event_manager.disconnect_ros(websocket)