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
# (추가) DB 모델 임포트
from app.models.models import Vehicle, PoliceCar

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
    # (수정) 리틀 엔디안으로 변경
    header = struct.pack('<BB', MessageType.NACK_ERROR, error_code)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

def _create_front_error_packet(error_code: FrontErrorCode) -> bytes:
    """프론트엔드로 보낼 시스템 에러 패킷을 생성합니다."""
    # (수정) 리틀 엔디안으로 변경
    header = struct.pack('<BB', MessageType.SYSTEM_ERROR, error_code)
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
        msg_type, vehicle_id, vehicle_type, car_name_bytes, received_hmac = struct.unpack('<BIB10s16s', data)

        if msg_type != MessageType.REGISTER_REQUEST:
            raise ValueError("Invalid message type")

        data_to_verify = data[:16]

        print("\n--- HMAC DEBUG START ---")
        print(f"Received Full Packet ({len(data)} bytes) : {data.hex()}")
        # (추가) 패킷의 마지막 16바이트(HMAC이어야 하는 부분) 출력
        print(f"Last 16 Bytes of Packet      : {data[-16:].hex()}")
        print(f"Data to Verify (First 16)    : {data_to_verify.hex()}")
        
        calculated_hmac = _calculate_hmac(data_to_verify)
        print(f"HMAC Calculated by Server    : {calculated_hmac.hex()}")
        print(f"HMAC Received from Client    : {received_hmac.hex()}")
        print("--- HMAC DEBUG END ---\n")

        if not hmac.compare_digest(calculated_hmac, received_hmac):
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
        print(f"DEBUG (register): Returning from validation error -> {result}")
        return result

    async with AsyncSessionMaker() as db_session:
        try:
            if await is_car_name_exists(db_session, request_data.car_name):
                ros_response = _create_ros_error_packet(RosErrorCode.DUPLICATE_NAME)
                result = (ros_response, None)
                print(f"DEBUG (register): Returning from duplicate name error -> {result}")
                return result
            
            vehicle_type_enum = VehicleTypeEnum.POLICE if request_data.vehicle_type == 0 else VehicleTypeEnum.RUNNER
            
            # --- ⬇️ (수정) PoliceCar 레코드 생성 로직 추가 ---
            new_vehicle_instance = Vehicle(
                vehicle_id=request_data.vehicle_id,
                vehicle_type=vehicle_type_enum,
                car_name=request_data.car_name,
            )

            if vehicle_type_enum == VehicleTypeEnum.POLICE:
                police_car_instance = PoliceCar()
                new_vehicle_instance.police_car = police_car_instance
            
            # 참고: save_vehicle 함수는 dict가 아닌 model instance를 받도록 수정해야 할 수 있습니다.
            new_vehicle = await save_vehicle(db_session, vehicle_instance=new_vehicle_instance)
            # --- ⬆️ (수정) PoliceCar 레코드 생성 로직 추가 ---

            event_data = VehicleRegisteredEvent(
                id=new_vehicle.id,
                vehicle_id=new_vehicle.vehicle_id,
                vehicle_type=request_data.vehicle_type,
                car_name=request_data.car_name
            )

        except Exception as e:
            print(f"Database Error: {e}")
            ros_response = _create_ros_error_packet(RosErrorCode.INVALID_DATA) # DB 문제는 INVALID_DATA가 더 적합
            front_event = _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)
            result = (ros_response, front_event)
            print(f"DEBUG (register): Returning from DB error -> {result}")
            return result

    # (수정) 리틀 엔디안으로 변경
    ros_response = struct.pack('<BI', MessageType.REGISTER_SUCCESS, event_data.vehicle_id)
    car_name_padded = event_data.car_name.encode('utf-8').ljust(10, b'\x00')
    event_header = struct.pack('<BIIB10s',
                               MessageType.EVENT_VEHICLE_REGISTERED,
                               event_data.id,
                               event_data.vehicle_id,
                               event_data.vehicle_type,
                               car_name_padded)
    event_hmac = _calculate_hmac(event_header)
    front_event = event_header + event_hmac

    result = (ros_response, front_event)
    print(f"DEBUG (register): Returning on success -> {result}")
    return result

async def handle_location_update(data: bytes) -> Optional[bytes]:
    try:
        if len(data) != 28:
            print(f"Invalid location packet size: {len(data)} bytes. Ignoring.")
            return None

        # (수정) 리틀 엔디안으로 변경
        ros_vehicle_id, pos_x, pos_y, received_hmac = struct.unpack('<Iff16s', data)

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
        return None

    async with AsyncSessionMaker() as db_session:
        try:
            success = await save_vehicle_location(db_session, location_data=request_data)
            if not success:
                print(f"DEBUG (location): Vehicle not found in DB for ID {request_data.vehicle_id}")
                return None

            event_data = VehicleLocationBroadcast(
                vehicle_id=request_data.vehicle_id,
                position_x=request_data.position_x,
                position_y=request_data.position_y
            )
        except Exception as e:
            print(f"Database Error on location save: {e}")
            return _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)

    # (수정) 리틀 엔디안으로 변경
    event_header = struct.pack('<BIff',
                               MessageType.POSITION_BROADCAST_2D,
                               event_data.vehicle_id,
                               event_data.position_x,
                               event_data.position_y)
    event_hmac = _calculate_hmac(event_header)
    return event_header + event_hmac


async def handle_vehicle_status_update(data: bytes) -> Tuple[Optional[bytes], Optional[bytes]]:
    try:
        if len(data) != 24:
            raise struct.error("Incorrect packet size for status update")

        # (수정) 리틀 엔디안으로 변경
        msg_type, vehicle_id, fuel, collision, status, received_hmac = struct.unpack('<BIBBB16s', data)

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
        return (ros_response, None)

    async with AsyncSessionMaker() as db_session:
        try:
            updated_police_car = await update_vehicle_status(db_session, status_data=request_data)
            if not updated_police_car:
                ros_response = _create_ros_error_packet(RosErrorCode.INVALID_DATA)
                return (ros_response, None)

            event_data = VehicleStatusBroadcast(
                vehicle_id=request_data.vehicle_id,
                collision_count=updated_police_car.collision_count,
                status_enum=request_data.status_enum,
                fuel=updated_police_car.fuel
            )
        except Exception as e:
            print(f"Database Error on status update: {e}")
            ros_response = _create_ros_error_packet(RosErrorCode.INVALID_DATA)
            front_event = _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)
            return (ros_response, front_event)

    # (수정) 리틀 엔디안으로 변경
    event_header = struct.pack('<BIBBB',
                               MessageType.STATE_UPDATE,
                               event_data.vehicle_id,
                               event_data.collision_count,
                               event_data.status_enum,
                               event_data.fuel)
    event_hmac = _calculate_hmac(event_header)
    front_event = event_header + event_hmac
    
    # ROS는 상태 업데이트에 대한 성공 응답이 필요 없다고 가정
    return (None, front_event)

# (수정) 맵 이벤트 관련 함수들도 모두 리틀 엔디안으로 변경
def create_start_tracking_packet(event_data: StartTrackingEvent) -> bytes:
    header = struct.pack('<BI', MessageType.EVENT_TRACE_START, event_data.runner_id)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

def create_capture_success_packet(event_data: CaptureSuccessEvent) -> bytes:
    header = struct.pack('<BII', MessageType.EVENT_CATCH, event_data.catcher_id, event_data.runner_id)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val

def create_catch_failed_packet(event_data: CatchFailedEvent) -> bytes:
    header = struct.pack('<BII', MessageType.EVENT_CATCH_FAILED, event_data.police_id, event_data.runner_id)
    hmac_val = _calculate_hmac(header)
    return header + hmac_val