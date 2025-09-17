# app/services/websocket_service.py

import logging
import struct
import hmac
import hashlib
import os
from typing import Tuple, Optional
from dotenv import load_dotenv

# --- 최종 확정된 ws_codes 임포트 ---
from app.common.ws_codes import MessageType, ErrorMessageType, ErrorCode
from app.db import (
    is_car_name_exists, save_vehicle, save_vehicle_location, update_vehicle_status,
    get_vehicle_by_ros_id, save_event, AsyncSessionMaker
)
from app.models.enums import VehicleTypeEnum, PoliceCarStatusEnum, EventStatus
from app.models.models import Vehicle, PoliceCar

# --- 로거 설정 ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- 환경 변수 및 HMAC 설정 ---
load_dotenv()
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError("HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다.")
SECRET_KEY = SECRET_key_str.encode('utf-8')

# --- 반환 타입 명시 (일관성을 위해 사용) ---
HandlerResult = Tuple[Optional[bytes], Optional[bytes], Optional[bytes]]

# ===================================================================
# 헬퍼 함수: 패킷 생성 및 HMAC 계산
# ===================================================================

def _calculate_hmac(data: bytes) -> bytes:
    """주어진 데이터의 HMAC을 계산합니다."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

def _create_ros_error_packet(error_code: ErrorCode) -> bytes:
    """ROS 클라이언트에게 보낼 NACK 에러 패킷을 생성합니다."""
    header = struct.pack('<BB', ErrorMessageType.NACK_ERROR, error_code.value)
    return header + _calculate_hmac(header)

# ===================================================================
# ROS 요청 처리 핸들러
# 각 함수는 (ROS응답, 프론트_브로드캐스트, ROS_브로드캐스트) 튜플을 반환합니다.
# ===================================================================

async def handle_vehicle_registration(data: bytes) -> HandlerResult:
    """차량 등록 요청(0xA0)을 처리합니다."""
    try:
        # 명세서에 따른 언패킹
        _, vehicle_id, vehicle_type_int, car_name_bytes, received_hmac = struct.unpack('<BIB10s16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:16]), received_hmac):
            raise ValueError("HMAC 검증 실패")
        car_name = car_name_bytes.decode('utf-8').strip('\x00')
        vehicle_type = VehicleTypeEnum.POLICE if vehicle_type_int == 0 else VehicleTypeEnum.RUNNER

    except (struct.error, ValueError, UnicodeDecodeError) as e:
        logging.warning(f"[{hex(MessageType.REGISTER_REQUEST)}] 등록 패킷 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(ErrorCode.INVALID_FORMAT), None, None)

    async with AsyncSessionMaker() as db_session:
        if await is_car_name_exists(db_session, car_name):
            logging.warning(f"[{hex(MessageType.REGISTER_REQUEST)}] 차량 이름 중복: '{car_name}'")
            return (_create_ros_error_packet(ErrorCode.DUPLICATE_NAME), None, None)

        new_vehicle = Vehicle(vehicle_id=vehicle_id, vehicle_type=vehicle_type, car_name=car_name)
        if vehicle_type == VehicleTypeEnum.POLICE:
            new_vehicle.police_car = PoliceCar()
        await save_vehicle(db_session, vehicle_instance=new_vehicle)
        await db_session.refresh(new_vehicle)
        logging.info(f"[DB] 새 차량 등록 성공: id={new_vehicle.id}, name='{new_vehicle.car_name}'")

        ros_broadcast_event = None
        if new_vehicle.vehicle_type == VehicleTypeEnum.RUNNER:
            await save_event(db_session, {"status": EventStatus.RUN, "runner_id": new_vehicle.id})
            run_header = struct.pack('<BI', MessageType.EVENT_RUN, new_vehicle.vehicle_id)
            ros_broadcast_event = run_header + _calculate_hmac(run_header)
            logging.info(f"[BCAST->ROS] 추적 시작({hex(MessageType.EVENT_RUN)}) 이벤트 생성")

        car_name_padded = new_vehicle.car_name.encode('utf-8').ljust(10, b'\x00')
        front_header = struct.pack('<BIIB10s', MessageType.EVENT_VEHICLE_REGISTERED, new_vehicle.id, new_vehicle.vehicle_id, vehicle_type_int, car_name_padded)
        front_event = front_header + _calculate_hmac(front_header)
        logging.info(f"[BCAST->FE] 차량 등록({hex(MessageType.EVENT_VEHICLE_REGISTERED)}) 이벤트 생성")

        return (None, front_event, ros_broadcast_event)


async def handle_location_update(data: bytes) -> HandlerResult:
    """차량 위치 정보(0x13)를 처리합니다."""
    try:
        _, vehicle_id, pos_x, pos_y, received_hmac = struct.unpack('<BIff16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:13]), received_hmac):
            raise ValueError("HMAC 검증 실패")
    except (struct.error, ValueError) as e:
        logging.warning(f"[{hex(MessageType.POSITION_BROADCAST)}] 위치 패킷 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(ErrorCode.INVALID_FORMAT), None, None)

    async with AsyncSessionMaker() as db_session:
        vehicle = await get_vehicle_by_ros_id(db_session, vehicle_id)
        if not vehicle:
            logging.warning(f"[{hex(MessageType.POSITION_BROADCAST)}] vehicle_id({vehicle_id})를 찾을 수 없음")
            return (_create_ros_error_packet(ErrorCode.INVALID_DATA), None, None)

        await save_vehicle_location(db_session, vehicle.id, pos_x, pos_y)

        positions_data = struct.pack('<Iff', vehicle.id, pos_x, pos_y)
        front_header = struct.pack('<BI', MessageType.POSITION_BROADCAST_2D, 1)
        front_event = front_header + positions_data + _calculate_hmac(front_header + positions_data)

        ros_broadcast_event = None
        if vehicle.vehicle_type == VehicleTypeEnum.RUNNER:
            ros_loc_header = struct.pack('<Iff', vehicle.vehicle_id, pos_x, pos_y)
            ros_broadcast_event = ros_loc_header + _calculate_hmac(ros_loc_header)
        
        return (None, front_event, ros_broadcast_event)


async def handle_vehicle_status_update(data: bytes) -> HandlerResult:
    """차량 상태 정보(0x12)를 처리합니다."""
    try:
        _, vehicle_id, fuel, collision, status_int, received_hmac = struct.unpack('<BIBBB16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:8]), received_hmac):
            raise ValueError("HMAC 검증 실패")
    except (struct.error, ValueError) as e:
        logging.warning(f"[{hex(MessageType.STATUS_UPDATE_REQUEST)}] 상태 패킷 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(ErrorCode.INVALID_FORMAT), None, None)
    
    async with AsyncSessionMaker() as db_session:
        vehicle = await get_vehicle_by_ros_id(db_session, vehicle_id)
        if not vehicle or vehicle.vehicle_type != VehicleTypeEnum.POLICE:
            return (_create_ros_error_packet(ErrorCode.INVALID_DATA), None, None)
        
        status_map = {i: s for i, s in enumerate(PoliceCarStatusEnum)}
        new_status_enum = status_map.get(status_int)
        if new_status_enum is None:
            return (_create_ros_error_packet(ErrorCode.INVALID_FORMAT), None, None)
            
        await update_vehicle_status(db_session, vehicle.id, fuel, collision, new_status_enum)

        front_header = struct.pack('<BIBBB', MessageType.STATE_UPDATE, vehicle.id, collision, status_int, fuel)
        front_event = front_header + _calculate_hmac(front_header)
        
        return (None, front_event, None)


async def handle_incoming_event(data: bytes) -> HandlerResult:
    """검거 이벤트(0xFE, 0xFD)를 처리합니다."""
    try:
        message_type = data[0]
        event_status_map = {
            MessageType.EVENT_CATCH: EventStatus.CATCH,
            MessageType.EVENT_CATCH_FAILED: EventStatus.FAILED,
        }
        event_type = event_status_map.get(message_type)
        if event_type is None:
            raise ValueError("알 수 없는 이벤트 타입")
            
        _, catcher_ros_id, runner_ros_id, received_hmac = struct.unpack('<BII16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:9]), received_hmac):
            raise ValueError("HMAC 검증 실패")

    except (struct.error, ValueError) as e:
        logging.warning(f"[EVENT] 이벤트 패킷 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(ErrorCode.INVALID_FORMAT), None, None)

    async with AsyncSessionMaker() as db_session:
        catcher = await get_vehicle_by_ros_id(db_session, catcher_ros_id)
        runner = await get_vehicle_by_ros_id(db_session, runner_ros_id)
        if not catcher or not runner:
            return (_create_ros_error_packet(ErrorCode.INVALID_DATA), None, None)
        
        await save_event(db_session, {"status": event_type, "catcher_id": catcher.id, "runner_id": runner.id})
        logging.info(f"[DB] {event_type.name} 이벤트 저장 완료")
        
        # 프론트와 ROS 모두에게 동일한 포맷으로 브로드캐스트
        event_header = struct.pack('<BII', message_type, catcher.id, runner.id)
        broadcast_packet = event_header + _calculate_hmac(event_header)
        logging.info(f"[BCAST->ALL] {event_type.name}({hex(message_type)}) 이벤트 생성")

        return (None, broadcast_packet, broadcast_packet)