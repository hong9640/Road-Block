# app/routers/websocket_router.py
# 웹소켓 라우터
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import traceback
import struct
from typing import Dict, List, Any

# --- 프로젝트 내부 모듈 import ---
from app.services import websocket_service
from app.common.ws_codes import MessageType
from app.db import AsyncSessionMaker, get_all_vehicles, get_all_events
from app.services.websocket_service import _calculate_hmac
from app.models.enums import EventStatus, VehicleTypeEnum, PoliceCarStatusEnum
from app.schemas import websocket_schema

# --- 연결 관리자 클래스 ---
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
        # 순회 중 리스트가 변경되어도 안전하도록 리스트의 복사본을 사용
        for connection in list(self.active_connections):
            try:
                await connection.send_bytes(message)
            except (WebSocketDisconnect, RuntimeError):
                self.disconnect(connection)

# --- 전역 매니저 객체 생성 ---
vehicle_manager = ConnectionManager()
event_manager = ConnectionManager()
# 서비스 계층에서 매니저 객체에 접근할 수 있도록 딕셔너리로 묶어 전달
managers: Dict[str, Any] = {"vehicle": vehicle_manager, "event": event_manager}
router = APIRouter(tags=["WebSockets"])


# ===================================================================
#               VEHICLE WEBSOCKET (/ws/vehicles)
# ===================================================================

async def send_initial_vehicle_data(websocket: WebSocket):
    """새로 연결된 클라이언트에게 DB의 모든 차량 및 관련 정보를 전송합니다."""
    print("새 클라이언트에게 기존 차량 데이터를 전송합니다.")
    async with AsyncSessionMaker() as db_session:
        all_vehicles = await get_all_vehicles(db_session)
        
        for vehicle in all_vehicles:
            # 1. 차량 기본 정보 전송 (0xA2)
            car_name_padded = vehicle.car_name.encode('utf-8').ljust(10, b'\x00')
            vehicle_type_int = 0 if vehicle.vehicle_type == VehicleTypeEnum.POLICE else 1
            reg_header = struct.pack('<BIIB10s', MessageType.EVENT_VEHICLE_REGISTERED, vehicle.id, vehicle.vehicle_id, vehicle_type_int, car_name_padded)
            await websocket.send_bytes(reg_header + _calculate_hmac(reg_header))

            # 2. 차량 최신 위치 정보 전송 (0xB1)
            if vehicle.locations:
                latest_location = vehicle.locations[-1]
                loc_header = struct.pack('<BIff', MessageType.POSITION_BROADCAST_2D, vehicle.id, latest_location.position_x, latest_location.position_y)
                await websocket.send_bytes(loc_header + _calculate_hmac(loc_header))

            # 3. 경찰차 상태 정보 전송 (0xD0)
            if vehicle.vehicle_type == VehicleTypeEnum.POLICE and vehicle.police_car:
                police_status = vehicle.police_car
                status_map = {
                    PoliceCarStatusEnum.NORMAL: 0,
                    PoliceCarStatusEnum.HALF_DESTROYED: 1,
                    PoliceCarStatusEnum.COMPLETE_DESTROYED: 2,
                }
                status_int = status_map.get(police_status.status, 0)
                status_header = struct.pack('<BIBBB', MessageType.STATE_UPDATE, vehicle.vehicle_id, police_status.collision_count, status_int, police_status.fuel)
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
                ros_response, front_event = await websocket_service.handle_vehicle_registration(data)
                if ros_response: await websocket.send_bytes(ros_response)
                if front_event: await vehicle_manager.broadcast(front_event)
            elif data_len == 28:
                front_event = await websocket_service.handle_location_update(data, managers)
                if front_event: await vehicle_manager.broadcast(front_event)
            elif data_len == 24:
                ros_response, front_event = await websocket_service.handle_vehicle_status_update(data, managers)
                if ros_response: await websocket.send_bytes(ros_response)
                if front_event: await vehicle_manager.broadcast(front_event)
            else:
                print(f"[/ws/vehicles] 정의되지 않은 길이의 패킷 수신: {data_len} bytes.")
    except (WebSocketDisconnect, RuntimeError):
        print("클라이언트 연결이 끊어져 작업을 중단합니다.")
        vehicle_manager.disconnect(websocket)
    except Exception as e:
        print(f"[/ws/vehicles] 처리되지 않은 웹소켓 에러 발생: {e}")
        traceback.print_exc()
        vehicle_manager.disconnect(websocket)

# ===================================================================
#                 EVENT WEBSOCKET (/ws/events)
# ===================================================================
async def send_initial_event_data(websocket: WebSocket):
    print("새 클라이언트에게 기존 이벤트 데이터를 전송합니다.")
    async with AsyncSessionMaker() as db_session:
        all_events = await get_all_events(db_session)
        for event in all_events:
            binary_packet = None
            # catcher 또는 runner가 없는 이벤트는 건너뜁니다 (DB 데이터 정합성)
            if not event.catcher or not event.runner:
                continue
            
            if event.status == EventStatus.CATCH:
                event_model = websocket_schema.CaptureSuccessEvent(catcher_id=event.catcher.vehicle_id, runner_id=event.runner.vehicle_id)
                binary_packet = websocket_service.create_capture_success_packet(event_model)
            elif event.status == EventStatus.RUN:
                event_model = websocket_schema.CatchFailedEvent(police_id=event.catcher.vehicle_id, runner_id=event.runner.vehicle_id)
                binary_packet = websocket_service.create_catch_failed_packet(event_model)
            if binary_packet:
                await websocket.send_bytes(binary_packet)
    print(f"기존 이벤트 데이터 전송 완료: 총 {len(all_events)}건")

@router.websocket("/ws/events")
async def unified_event_websocket(websocket: WebSocket):
    await event_manager.connect(websocket)
    try:
        await send_initial_event_data(websocket)
        while True:
            await websocket.receive_text()
    except (WebSocketDisconnect, RuntimeError):
        event_manager.disconnect(websocket)
    except Exception as e:
        print(f"[/ws/events] 웹소켓 에러 발생: {e}")
        traceback.print_exc()
        event_manager.disconnect(websocket)