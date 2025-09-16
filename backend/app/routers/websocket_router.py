# app/routers/websocket_router.py

from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import traceback
import struct
from typing import Dict, List, Any

# --- 프로젝트 내부 모듈 import ---
from app.services import websocket_service
from app.common.ws_codes import MessageType
from app.db import AsyncSessionMaker, get_all_events, get_all_vehicles
from app.services.websocket_service import _calculate_hmac
from app.models.enums import EventStatus, VehicleTypeEnum, PoliceCarStatusEnum
from app.schemas import websocket_schema

# --- 연결 관리자 클래스 (변경 없음) ---
class ConnectionManager:
    """웹소켓 연결을 관리하는 범용 매니저"""
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"클라이언트 연결. 총 클라이언트: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            print(f"클라이언트 연결 해제. 총 클라이언트: {len(self.active_connections)}")

    async def broadcast(self, message: bytes):
        for connection in list(self.active_connections):
            try:
                await connection.send_bytes(message)
            except (WebSocketDisconnect, RuntimeError):
                self.disconnect(connection)

# --- 전역 매니저 객체 생성 ---
vehicle_manager = ConnectionManager()
event_manager = ConnectionManager()
managers: Dict[str, Any] = {"vehicle": vehicle_manager, "event": event_manager}
router = APIRouter(tags=["WebSockets"])

# ===================================================================
#               VEHICLE WEBSOCKET (/ws/vehicles)
# ===================================================================

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

@router.websocket("/ws/vehicles")
async def unified_vehicle_websocket(websocket: WebSocket):
    await vehicle_manager.connect(websocket)
    try:
        await send_initial_vehicle_data(websocket)
        while True:
            data = await websocket.receive_bytes()
            data_len = len(data)
            if data_len == 32:
                ros_response, front_event = await websocket_service.handle_vehicle_registration(data, managers)
                if ros_response: await websocket.send_bytes(ros_response)
                if front_event: await vehicle_manager.broadcast(front_event)
            elif data_len == 28:
                ros_response, front_event = await websocket_service.handle_location_update(data, managers)
                if front_event: await vehicle_manager.broadcast(front_event)
            elif data_len == 24:
                ros_response, front_event = await websocket_service.handle_vehicle_status_update(data, managers)
                if ros_response: await websocket.send_bytes(ros_response)
                if front_event: await vehicle_manager.broadcast(front_event)
            else:
                print(f"[/ws/vehicles] 정의되지 않은 길이의 패킷 수신: {data_len} bytes.")
    except (WebSocketDisconnect, RuntimeError):
        vehicle_manager.disconnect(websocket)
    except Exception as e:
        print(f"[/ws/vehicles] 처리되지 않은 웹소켓 에러 발생: {e}")
        traceback.print_exc()
        vehicle_manager.disconnect(websocket)

# ===================================================================
#                 EVENT WEBSOCKET (/ws/events)
# ===================================================================

async def send_initial_event_data(websocket: WebSocket):
    """
    ✨ 여기가 수정된 부분입니다.
    과거 이벤트 목록은 실시간 이벤트와 동일하게 vehicle_id를 사용해야 일관성이 유지됩니다.
    """
    print("새 클라이언트에게 기존 이벤트 데이터를 전송합니다.")
    async with AsyncSessionMaker() as db_session:
        all_events = await get_all_events(db_session)
        for event in all_events:
            binary_packet = None
            if not hasattr(event, 'runner') or not event.runner: continue
            
            # 검거 성공 이벤트 (0xFE)
            if event.status == EventStatus.CATCH and hasattr(event, 'catcher') and event.catcher:
                # ✨ FIX: 임베디드와의 호환성을 위해 vehicle_id 사용
                event_model = websocket_schema.CaptureSuccessEvent(
                    catcher_id=event.catcher.id, 
                    runner_id=event.runner.id
                )
                binary_packet = websocket_service.create_capture_success_packet(event_model)
            
            # 검거 실패 이벤트 (0xFD)
            elif event.status == EventStatus.FAILED and hasattr(event, 'catcher') and event.catcher:
                # ✨ FIX: 임베디드와의 호환성을 위해 vehicle_id 사용
                event_model = websocket_schema.CatchFailedEvent(
                    police_id=event.catcher.id, 
                    runner_id=event.runner.id
                )
                binary_packet = websocket_service.create_catch_failed_packet(event_model)

            # 추적 시작 이벤트 (0xF0)
            elif event.status == EventStatus.RUN:
                # ✨ FIX: 임베디드와의 호환성을 위해 vehicle_id 사용
                event_model = websocket_schema.StartTrackingEvent(
                    runner_id=event.runner.id
                )
                binary_packet = websocket_service.create_start_tracking_packet(event_model)

            if binary_packet:
                await websocket.send_bytes(binary_packet)
                
    print(f"기존 이벤트 데이터 전송 완료: 총 {len(all_events)}건")

@router.websocket("/ws/events")
async def unified_event_websocket(websocket: WebSocket):
    await event_manager.connect(websocket)
    try:
        await send_initial_event_data(websocket)
        while True:
            data = await websocket.receive_bytes()
            await websocket_service.handle_incoming_event(data, managers)
    except (WebSocketDisconnect, RuntimeError):
        event_manager.disconnect(websocket)
    except Exception as e:
        print(f"[/ws/events] 웹소켓 에러 발생: {e}")
        traceback.print_exc()
        event_manager.disconnect(websocket)