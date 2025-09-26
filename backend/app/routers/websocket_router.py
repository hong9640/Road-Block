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
                        try:
                            # <BIBf -> type(1), id(4), status(1), timestamp(4) = 10바이트
                            _, runner_id, status_int, _ = struct.unpack('<BIBf', event_packet[:10])
                            event_log.info(f"BROADCAST EVENT [TRACE_START]: runner_id={runner_id}")
                        except struct.error:
                            event_log.info(f"BROADCAST EVENT [TRACE_START]: (패킷 해석 실패)")
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
                    try:
                        # <BII -> type(1), catcher_id(4), runner_id(4) = 9바이트
                        msg_type, catcher_id, runner_id = struct.unpack('<BII', front_broadcast[:9])
                        log_event_type = "CATCH" if msg_type == MessageType.EVENT_CATCH else "FAILED"
                        event_log.info(f"BROADCAST EVENT [{log_event_type}]: catcher_id={catcher_id}, runner_id={runner_id}")
                    except struct.error:
                        event_log.info(f"BROADCAST EVENT [CATCH/FAILED]: (패킷 해석 실패)")
                    await event_manager.broadcast_to_front(front_broadcast)
                if ros_broadcast:
                    await event_manager.broadcast_to_all_ros(ros_broadcast)

    except (WebSocketDisconnect, RuntimeError) as e:
        logging.warning(f"ROS 이벤트 클라이언트 연결 종료: {e}")
    finally:
        event_manager.disconnect_ros(websocket)