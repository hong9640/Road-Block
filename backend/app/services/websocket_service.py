import struct
import hmac
import hashlib
from typing import Tuple, Optional
import os
from dotenv import load_dotenv

from app.common.ws_codes import MessageType, RosErrorCode, FrontErrorCode
from app.schemas.websocket_schema import VehicleRegistrationRequest, VehicleRegisteredEvent

# --- (수정) 데이터베이스 관련 모듈 임포트 ---
# db.py에서 필요한 함수와 세션 메이커를 직접 가져옵니다.
from app.db import is_car_name_exists, save_vehicle, AsyncSessionMaker
from app.models.enums import VehicleTypeEnum


# --- .env 파일에서 환경 변수 로드 ---
load_dotenv()
SECRET_KEY_STR = os.getenv("HMAC_SECRET_KEY")
if not SECRET_KEY_STR:
    raise ValueError("HMAC_SECRET_KEY 환경 변수가 설정되지 않았습니다. .env 파일을 확인해주세요.")
SECRET_KEY = SECRET_KEY_STR.encode('utf-8')


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
    print(f"--- Received WebSocket Data ---")
    print(f"Data Type: {type(data)}")
    if isinstance(data, bytes):
        print(f"Data Length: {len(data)} bytes")
        print(f"Data Hex: {data.hex()}")
    else:
        print(f"Data Content: {data}")
    print("-----------------------------")
    # --------------------------------
    # 1. ROS 요청(0xA0) 파싱 및 검증
    try:
        if len(data) != 28:
            raise struct.error("Incorrect packet size")
        
        msg_type, vehicle_type, car_name_bytes, received_hmac = struct.unpack('>BB10s16s', data)

        if msg_type != MessageType.REGISTER_REQUEST:
            raise ValueError("Invalid message type")

        data_to_verify = data[:12]
        if not hmac.compare_digest(_calculate_hmac(data_to_verify), received_hmac):
            raise ValueError("HMAC validation failed")

        request_data = VehicleRegistrationRequest(
            vehicle_type=vehicle_type,
            car_name=car_name_bytes.decode('utf-8').strip('\x00')
        )

    except (struct.error, ValueError) as e:
        print(f"Validation Error: {e}")
        ros_response = _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
        return ros_response, None

    # 2. 비즈니스 로직 (비동기 DB 처리)
    async with AsyncSessionMaker() as db_session:
        try:
            # db.py에 정의된 함수를 직접 비동기 호출합니다.
            if await is_car_name_exists(db_session, request_data.car_name):
                ros_response = _create_ros_error_packet(RosErrorCode.DUPLICATE_NAME)
                return ros_response, None
            vehicle_type_enum = VehicleTypeEnum.POLICE if request_data.vehicle_type == 0 else VehicleTypeEnum.RUNNER

            # save_vehicle 함수에 Pydantic 스키마 대신, 변환된 데이터가 포함된 딕셔너리를 전달합니다.
            db_save_data = {
                "car_name": request_data.car_name,
                "vehicle_type": vehicle_type_enum
            }
            new_vehicle = await save_vehicle(db_session, vehicle_data=db_save_data)
            
            # (수정) models.py의 필드명(vehicle_id)을 정확히 사용합니다.
            new_vehicle_id = new_vehicle.vehicle_id
            
            event_data = VehicleRegisteredEvent(
                vehicle_id=new_vehicle_id,
                vehicle_type=request_data.vehicle_type,
                car_name=request_data.car_name
            )

        except Exception as e:
            print(f"Database Error: {e}")
            ros_response = _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
            front_event = _create_front_error_packet(FrontErrorCode.DATABASE_ERROR)
            return ros_response, front_event

    # 3. 성공 응답 패킷 생성
    ros_response = struct.pack('>BI', MessageType.REGISTER_SUCCESS, event_data.vehicle_id)

    car_name_padded = event_data.car_name.encode('utf-8').ljust(10, b'\x00')
    event_header = struct.pack('>BIB10s',
                               MessageType.EVENT_VEHICLE_REGISTERED,
                               event_data.vehicle_id,
                               event_data.vehicle_type,
                               car_name_padded)
    event_hmac = _calculate_hmac(event_header)
    front_event = event_header + event_hmac

    return ros_response, front_event

