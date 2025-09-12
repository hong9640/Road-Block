# app/services/websocket_service.py

import struct
import hmac
import hashlib
from typing import Tuple, Optional, Any, Dict
import os
import math
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select

# --- 프로젝트 내부 모듈 import ---
from app.common.ws_codes import MessageType, RosErrorCode, FrontErrorCode
from app.schemas import websocket_schema
from app.db import (
    is_car_name_exists, save_vehicle, save_vehicle_location, update_vehicle_status,
    get_vehicle_by_ros_id, get_all_vehicles, save_event, AsyncSessionMaker
)
from app.models.enums import VehicleTypeEnum, PoliceCarStatusEnum, EventStatus
from app.models.models import Vehicle, PoliceCar

# --- 환경 변수 및 HMAC 설정 ---
load_dotenv()
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError("HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다.")
SECRET_KEY = SECRET_key_str.encode('utf-8')

def _calculate_hmac(data: bytes) -> bytes:
    """주어진 데이터로 HMAC-SHA256 값을 계산합니다 (16바이트로 자름)."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

# --- 패킷 생성 헬퍼 ---
def _create_ros_error_packet(error_code: RosErrorCode) -> bytes:
    """ROS(임베디드)로 보낼 에러(NACK) 패킷을 생성합니다."""
    header = struct.pack('<BB', MessageType.NACK_ERROR, error_code.value)
    return header + _calculate_hmac(header)

def _create_front_error_packet(error_code: FrontErrorCode) -> bytes:
    """프론트엔드로 보낼 시스템 에러 패킷을 생성합니다."""
    header = struct.pack('<BB', MessageType.SYSTEM_ERROR, error_code.value)
    return header + _calculate_hmac(header)

def create_start_tracking_packet(event_data: websocket_schema.StartTrackingEvent) -> bytes:
    """추적 시작 이벤트 바이너리 패킷(0xF0)을 생성합니다."""
    header = struct.pack('<BI', MessageType.EVENT_TRACE_START, event_data.runner_id)
    return header + _calculate_hmac(header)

def create_capture_success_packet(event_data: websocket_schema.CaptureSuccessEvent) -> bytes:
    """검거 성공 이벤트 바이너리 패킷(0xFE)을 생성합니다."""
    header = struct.pack('<BII', MessageType.EVENT_CATCH, event_data.catcher_id, event_data.runner_id)
    return header + _calculate_hmac(header)

def create_catch_failed_packet(event_data: websocket_schema.CatchFailedEvent) -> bytes:
    """검거 실패 이벤트 바이너리 패킷(0xFD)을 생성합니다."""
    header = struct.pack('<BII', MessageType.EVENT_CATCH_FAILED, event_data.police_id, event_data.runner_id)
    return header + _calculate_hmac(header)


# --- 게임 이벤트 발생 로직 ---
async def trigger_event_broadcasts(session: AsyncSession, event_type: str, police_ros_id: int, runner_ros_id: int, managers: Dict[str, Any]):
    """
    검거/실패 이벤트를 생성하고 DB에 저장한 뒤, 프론트엔드와 임베디드에 모두 브로드캐스트합니다.
    """
    binary_packet, db_event_data = None, None
    
    catcher_vehicle = await get_vehicle_by_ros_id(session, police_ros_id)
    runner_vehicle = await get_vehicle_by_ros_id(session, runner_ros_id)

    if not catcher_vehicle or not runner_vehicle:
        print("이벤트 발생 실패: police 또는 runner 차량을 찾을 수 없습니다.")
        return

    if event_type == "capture_success":
        event_model = websocket_schema.CaptureSuccessEvent(catcher_id=police_ros_id, runner_id=runner_ros_id)
        binary_packet = create_capture_success_packet(event_model)
        db_event_data = {"status": EventStatus.CATCH, "catcher_id": catcher_vehicle.id, "runner_id": runner_vehicle.id}

    elif event_type == "capture_failed":
        event_model = websocket_schema.CatchFailedEvent(police_id=police_ros_id, runner_id=runner_ros_id)
        binary_packet = create_catch_failed_packet(event_model)
        db_event_data = {"status": EventStatus.RUN, "catcher_id": catcher_vehicle.id, "runner_id": runner_vehicle.id}
        
    if binary_packet and db_event_data:
        await save_event(session, db_event_data)
        if 'event' in managers: await managers['event'].broadcast(binary_packet)
        if 'vehicle' in managers: await managers['vehicle'].broadcast(binary_packet)
        print(f"--- 이벤트 발생 및 전파 완료: {event_type} ---")


# --- 메인 서비스 핸들러 ---

async def handle_vehicle_registration(data: bytes) -> Tuple[bytes, Optional[bytes]]:
    """차량 등록 요청을 처리하고 (ROS 응답, 프론트엔드 이벤트)를 반환합니다."""
    try:
        if len(data) != 32: raise struct.error("Incorrect packet size")
        msg_type, vehicle_id, vehicle_type, car_name_bytes, received_hmac = struct.unpack('<BIB10s16s', data)
        if msg_type != MessageType.REGISTER_REQUEST: raise ValueError("Invalid message type")
        
        data_to_verify = data[:16]
        if not hmac.compare_digest(_calculate_hmac(data_to_verify), received_hmac):
            raise ValueError("HMAC validation failed")

        request_data = websocket_schema.VehicleRegistrationRequest(
            vehicle_id=vehicle_id,
            vehicle_type=vehicle_type,
            car_name=car_name_bytes.decode('utf-8').strip('\x00')
        )
    except (struct.error, ValueError) as e:
        print(f"Validation Error: {e}")
        return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

    async with AsyncSessionMaker() as db_session:
        try:
            if await is_car_name_exists(db_session, request_data.car_name):
                return (_create_ros_error_packet(RosErrorCode.DUPLICATE_NAME), None)
            
            vehicle_type_enum = VehicleTypeEnum.POLICE if request_data.vehicle_type == 0 else VehicleTypeEnum.RUNNER
            
            new_vehicle_instance = Vehicle(
                vehicle_id=request_data.vehicle_id,
                vehicle_type=vehicle_type_enum,
                car_name=request_data.car_name,
            )

            if vehicle_type_enum == VehicleTypeEnum.POLICE:
                new_vehicle_instance.police_car = PoliceCar()
            
            new_vehicle = await save_vehicle(db_session, vehicle_instance=new_vehicle_instance)
            
            event_data = websocket_schema.VehicleRegisteredEvent(
                id=new_vehicle.id,
                vehicle_id=new_vehicle.vehicle_id,
                vehicle_type=request_data.vehicle_type,
                car_name=request_data.car_name
            )
        except Exception as e:
            print(f"Database Error: {e}")
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), _create_front_error_packet(FrontErrorCode.DATABASE_ERROR))

    ros_response = struct.pack('<BI', MessageType.REGISTER_SUCCESS, event_data.vehicle_id)
    car_name_padded = event_data.car_name.encode('utf-8').ljust(10, b'\x00')
    event_header = struct.pack('<BIIB10s',
                               MessageType.EVENT_VEHICLE_REGISTERED,
                               event_data.id,
                               event_data.vehicle_id,
                               event_data.vehicle_type,
                               car_name_padded)
    front_event = event_header + _calculate_hmac(event_header)
    return (ros_response, front_event)

async def handle_location_update(data: bytes, managers: Dict[str, Any]) -> Optional[bytes]:
    """차량 위치 정보 업데이트를 처리하고, 충돌 시 이벤트를 발생시킵니다."""
    try:
        if len(data) != 28: return None
        ros_vehicle_id, pos_x, pos_y, received_hmac = struct.unpack('<Iff16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:12]), received_hmac): return None
    except (struct.error, ValueError): return None

    async with AsyncSessionMaker() as db_session:
        vehicle = await get_vehicle_by_ros_id(db_session, ros_vehicle_id)
        if not vehicle: return None

        await save_vehicle_location(db_session, vehicle_pk_id=vehicle.id, pos_x=pos_x, pos_y=pos_y)

        if vehicle.vehicle_type == VehicleTypeEnum.RUNNER:
            all_vehicles = await get_all_vehicles(db_session)
            police_cars = [v for v in all_vehicles if v.vehicle_type == VehicleTypeEnum.POLICE and v.locations]
            for police in police_cars:
                latest_police_loc = police.locations[-1]
                distance = math.sqrt((pos_x - latest_police_loc.position_x)**2 + (pos_y - latest_police_loc.position_y)**2)
                if distance < 1.0: # 충돌 임계값 (조정 가능)
                    await trigger_event_broadcasts(db_session, "capture_success", police.vehicle_id, vehicle.vehicle_id, managers)
                    break
    
    event_header = struct.pack('<BIff', MessageType.POSITION_BROADCAST_2D, ros_vehicle_id, pos_x, pos_y)
    return event_header + _calculate_hmac(event_header)

async def handle_vehicle_status_update(data: bytes, managers: Dict[str, Any]) -> Tuple[Optional[bytes], Optional[bytes]]:
    """차량 상태 업데이트를 처리하고, 파괴 시 이벤트를 발생시킵니다."""
    try:
        if len(data) != 24: raise struct.error("Incorrect size")
        msg_type, vehicle_id, fuel, collision, status_int, received_hmac = struct.unpack('<BIBBB16s', data)
        if msg_type != MessageType.STATUS_UPDATE_REQUEST: raise ValueError("Invalid type")
        if not hmac.compare_digest(_calculate_hmac(data[:8]), received_hmac): raise ValueError("HMAC fail")
    except (struct.error, ValueError) as e:
        return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

    async with AsyncSessionMaker() as db_session:
        vehicle = await get_vehicle_by_ros_id(db_session, vehicle_id)
        if not vehicle or vehicle.vehicle_type != VehicleTypeEnum.POLICE:
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), None)

        status_map = {0: PoliceCarStatusEnum.NORMAL, 1: PoliceCarStatusEnum.HALF_DESTROYED, 2: PoliceCarStatusEnum.COMPLETE_DESTROYED}
        new_status_enum = status_map.get(status_int)
        if new_status_enum is None: return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

        updated_car = await update_vehicle_status(db_session, vehicle.id, fuel, collision, new_status_enum)
        if not updated_car: return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), None)
            
        if updated_car.status == PoliceCarStatusEnum.COMPLETE_DESTROYED:
            runners_result = await db_session.execute(select(Vehicle).where(Vehicle.vehicle_type == VehicleTypeEnum.RUNNER))
            active_runner = runners_result.scalars().first()
            if active_runner:
                await trigger_event_broadcasts(db_session, "capture_failed", vehicle.vehicle_id, active_runner.vehicle_id, managers)

    event_header = struct.pack('<BIBBB', MessageType.STATE_UPDATE, vehicle_id, collision, status_int, fuel)
    front_event = event_header + _calculate_hmac(event_header)
    return (None, front_event)