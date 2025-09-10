import struct
import hmac
import hashlib
from typing import Tuple, Optional
import os
from dotenv import load_dotenv

from app.common.ws_codes import MessageType, RosErrorCode, FrontErrorCode
from app.schemas.websocket_schema import (
    VehicleRegistrationRequest,
    VehicleRegisteredEvent,
    VehicleLocationUpdateRequest,
    VehicleLocationBroadcast,
    VehicleStatusUpdateRequest,
    VehicleStatusBroadcast,
    StartTrackingEvent,
    CaptureSuccessEvent,
    CatchFailedEvent
)
from app.db import (
    is_car_name_exists,
    save_vehicle,
    save_vehicle_location,
    update_vehicle_status,
    AsyncSessionMaker
)
from app.models.enums import VehicleTypeEnum

# --- .env 파일에서 환경 변수 로드 ---
load_dotenv()
SECRET_key_str = os.getenv("HMAC_SECRET_KEY")
if not SECRET_key_str:
    raise ValueError("HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일을 확인해주세요.")
SECRET_KEY = SECRET_key_str.encode('utf-8')

def _calculate_hmac(data: bytes) -> bytes:
    """주어진 데이터로 HMAC-SHA256 값을 계산합니다 (16바이트로 자름)."""
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

# --- 패킷 생성 헬퍼 함수 ---
def _create_ros_error_packet(error_code: RosErrorCode) -> bytes:
    """ROS(임베디드)로 보낼 에러(NACK) 패킷을 생성합니다."""
    header = struct.pack('>BB', MessageType.NACK_ERROR, error_code)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

def _create_front_error_packet(error_code: FrontErrorCode) -> bytes:
    """프론트엔드로 보낼 시스템 에러 패킷을 생성합니다."""
    header = struct.pack('>BB', MessageType.SYSTEM_ERROR, error_code)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

# --- 메인 서비스 핸들러 ---
async def handle_vehicle_registration(data: bytes) -> Tuple[bytes, Optional[bytes]]:
    """
    차량 등록 요청을 처리하고 (ROS 응답, 프론트엔드 이벤트)를 반환합니다.
    """
    try:
        if len(data) != 32:
            raise struct.error("Incorrect packet size")
        msg_type, vehicle_id, vehicle_type, car_name_bytes, received_hmac = struct.unpack('>BIB10s16s', data)

        if msg_type != MessageType.REGISTER_REQUEST:
            raise ValueError("Invalid message type")

        data_to_verify = data[:16]
        if not hmac.compare_digest(_calculate_hmac(data_to_verify), received_hmac):
            raise ValueError("HMAC validation failed")

        request_data = VehicleRegistrationRequest(
            vehicle_id=vehicle_id,
            vehicle_type=vehicle_type,
            car_name=car_name_bytes.decode('utf-8').strip('\x00')
        )

    except (struct.error, ValueError) as e:
        print(f"Validation Error: {e}")
        ros_response = _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
        result = (ros_response, None)
        # 🌟 DEBUG: 반환 값 추적
        print(f"DEBUG (register): Returning from validation error -> {result}")
        return result

    async with AsyncSessionMaker() as db_session:
        try:
            if await is_car_name_exists(db_session, request_data.car_name):
                ros_response = _create_ros_error_packet(RosErrorCode.DUPLICATE_NAME)
                result = (ros_response, None)
                # 🌟 DEBUG: 반환 값 추적
                print(f"DEBUG (register): Returning from duplicate name error -> {result}")
                return result

            vehicle_type_enum = VehicleTypeEnum.POLICE if request_data.vehicle_type == 0 else VehicleTypeEnum.RUNNER
            db_save_data = {
                "vehicle_id": request_data.vehicle_id,
                "vehicle_type": vehicle_type_enum,
                "car_name": request_data.car_name,
            }
            new_vehicle = await save_vehicle(db_session, vehicle_data=db_save_data)

            event_data = VehicleRegisteredEvent(
                id=new_vehicle.id,
                vehicle_id=new_vehicle.vehicle_id,
                vehicle_type=request_data.vehicle_type,
                car_name=request_data.car_name
            )

        except Exception as e:
            print(f"Database Error: {e}")
            ros_response = _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
            front_event = _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)
            result = (ros_response, front_event)
            # 🌟 DEBUG: 반환 값 추적
            print(f"DEBUG (register): Returning from DB error -> {result}")
            return result

    ros_response = struct.pack('>BI', MessageType.REGISTER_SUCCESS, event_data.vehicle_id)
    car_name_padded = event_data.car_name.encode('utf-8').ljust(10, b'\x00')
    event_header = struct.pack('>BIIB10s',
                               MessageType.EVENT_VEHICLE_REGISTERED,
                               event_data.id,
                               event_data.vehicle_id,
                               event_data.vehicle_type,
                               car_name_padded)
    event_hmac = _calculate_hmac(event_header)
    front_event = event_header + event_hmac

    result = (ros_response, front_event)
    # 🌟 DEBUG: 반환 값 추적
    print(f"DEBUG (register): Returning on success -> {result}")
    return result

async def handle_location_update(data: bytes) -> Optional[bytes]:
    """
    차량 위치 업데이트 요청을 처리하고 브로드캐스트할 이벤트 패킷을 반환합니다.
    """
    try:
        if len(data) != 28:
            print(f"Invalid location packet size: {len(data)} bytes. Ignoring.")
            result = None
            # 🌟 DEBUG: 반환 값 추적
            print(f"DEBUG (location): Returning from invalid packet size -> {result}")
            return result

        ros_vehicle_id, pos_x, pos_y, received_hmac = struct.unpack('>Iff16s', data)

        data_to_verify = data[:12]
        if not hmac.compare_digest(_calculate_hmac(data_to_verify), received_hmac):
            print("Location update HMAC validation failed. Ignoring.")
            return None

        request_data = VehicleLocationUpdateRequest(
            vehicle_id=ros_vehicle_id,
            position_x=pos_x,
            position_y=pos_y
        )

    except (struct.error, ValueError) as e:
        print(f"Location update validation Error: {e}. Ignoring.")
        result = None
        # 🌟 DEBUG: 반환 값 추적
        print(f"DEBUG (location): Returning from validation error -> {result}")
        return result

    async with AsyncSessionMaker() as db_session:
        try:
            success = await save_vehicle_location(db_session, location_data=request_data)
            if not success:
                result = None
                # 🌟 DEBUG: 반환 값 추적
                print(f"DEBUG (location): Returning because vehicle not found in DB -> {result}")
                return result

            event_data = VehicleLocationBroadcast(
                vehicle_id=request_data.vehicle_id,
                position_x=request_data.position_x,
                position_y=request_data.position_y
            )

        except Exception as e:
            print(f"Database Error on location save: {e}")
            result = _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)
            # 🌟 DEBUG: 반환 값 추적
            print(f"DEBUG (location): Returning from DB error -> {result}")
            return result

    event_header = struct.pack('>BIff',
                               MessageType.POSITION_BROADCAST_2D,
                               event_data.vehicle_id,
                               event_data.position_x,
                               event_data.position_y)
    event_hmac = _calculate_hmac(event_header)
    front_event = event_header + event_hmac
    
    # 🌟 DEBUG: 반환 값 추적
    print(f"DEBUG (location): Returning on success -> {front_event}")
    # (수정) 이 함수는 하나의 값만 반환해야 합니다.
    return front_event


async def handle_vehicle_status_update(data: bytes) -> Tuple[Optional[bytes], Optional[bytes]]:
    """
    차량 상태 업데이트 요청을 처리하고 (ROS 응답, 프론트엔드 브로드캐스트) 튜플을 반환합니다.
    """
    try:
        if len(data) != 24:
            raise struct.error("Incorrect packet size for status update")

        msg_type, vehicle_id, fuel, collision, status, received_hmac = struct.unpack('>BIBBB16s', data)

        if msg_type != MessageType.STATUS_UPDATE_REQUEST:
             raise ValueError("Invalid message type for status update")

        data_to_verify = data[:8]
        if not hmac.compare_digest(_calculate_hmac(data_to_verify), received_hmac):
            raise ValueError("HMAC validation failed for status update")

        request_data = VehicleStatusUpdateRequest(
            vehicle_id=vehicle_id,
            fuel=fuel,
            collision_count=collision,
            status_enum=status,
        )

    except (struct.error, ValueError) as e:
        print(f"Status Update Validation Error: {e}")
        ros_response = _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
        result = (ros_response, None)
        # 🌟 DEBUG: 반환 값 추적
        print(f"DEBUG (status): Returning from validation error -> {result}")
        return result

    async with AsyncSessionMaker() as db_session:
        try:
            updated_police_car = await update_vehicle_status(db_session, status_data=request_data)
            if not updated_police_car:
                ros_response = _create_ros_error_packet(RosErrorCode.INVALID_DATA)
                result = (ros_response, None)
                # 🌟 DEBUG: 반환 값 추적
                print(f"DEBUG (status): Returning because vehicle not found in DB -> {result}")
                return result

            event_data = VehicleStatusBroadcast(
                vehicle_id=request_data.vehicle_id,
                collision_count=updated_police_car.collision_count,
                status_enum=request_data.status_enum,
                fuel=updated_police_car.fuel
            )

        except Exception as e:
            print(f"Database Error on status update: {e}")
            ros_response = _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
            front_event = _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)
            result = (ros_response, front_event)
            # 🌟 DEBUG: 반환 값 추적
            print(f"DEBUG (status): Returning from DB error -> {result}")
            return result

    event_header = struct.pack('>BIBBB',
                               MessageType.STATE_UPDATE,
                               event_data.vehicle_id,
                               event_data.collision_count,
                               event_data.status_enum,
                               event_data.fuel)
    event_hmac = _calculate_hmac(event_header)
    front_event = event_header + event_hmac
    
    result = (None, front_event)
    # 🌟 DEBUG: 반환 값 추적
    print(f"DEBUG (status): Returning on success -> {result}")
    return result

def create_start_tracking_packet(event_data: StartTrackingEvent) -> bytes:
    """'추적 시작' 이벤트 패킷(0xF0)을 생성합니다."""
    header = struct.pack('>BI', MessageType.EVENT_TRACE_START, event_data.runner_id)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

def create_capture_success_packet(event_data: CaptureSuccessEvent) -> bytes:
    """'검거 성공' 이벤트 패킷(0xFE)을 생성합니다."""
    header = struct.pack('>BII', MessageType.EVENT_CATCH, event_data.catcher_id, event_data.runner_id)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

def create_catch_failed_packet(event_data: CatchFailedEvent) -> bytes:
    """'추적 실패' 이벤트 패킷(0xFD)을 생성합니다."""
    header = struct.pack('>BII', MessageType.EVENT_CATCH_FAILED, event_data.police_id, event_data.runner_id)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val
