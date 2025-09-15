# app/services/websocket_service.py
import traceback
import logging
import struct
import hmac
import hashlib
from typing import Tuple, Optional, Any, Dict
import os
import math
from dotenv import load_dotenv
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload
from sqlmodel import select

# --- 로거 설정 ---
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

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
    return hmac.new(SECRET_KEY, data, hashlib.sha256).digest()[:16]

# --- 패킷 생성 헬퍼 ---
def _create_ros_error_packet(error_code: RosErrorCode) -> bytes:
    header = struct.pack('<BB', MessageType.NACK_ERROR, error_code.value)
    return header + _calculate_hmac(header)

def _create_front_error_packet(error_code: FrontErrorCode) -> bytes:
    header = struct.pack('<BB', MessageType.SYSTEM_ERROR, error_code.value)
    return header + _calculate_hmac(header)

def create_start_tracking_packet(event_data: websocket_schema.StartTrackingEvent) -> bytes:
    header = struct.pack('<BI', MessageType.EVENT_TRACE_START, event_data.runner_id)
    return header + _calculate_hmac(header)

def create_capture_success_packet(event_data: websocket_schema.CaptureSuccessEvent) -> bytes:
    header = struct.pack('<BII', MessageType.EVENT_CATCH, event_data.catcher_id, event_data.runner_id)
    return header + _calculate_hmac(header)

def create_catch_failed_packet(event_data: websocket_schema.CatchFailedEvent) -> bytes:
    header = struct.pack('<BII', MessageType.EVENT_CATCH_FAILED, event_data.police_id, event_data.runner_id)
    return header + _calculate_hmac(header)

# --- 게임 이벤트 발생 로직 ---
async def create_and_save_event(session: AsyncSession, event_type: str, police_ros_id: int, runner_ros_id: int) -> Optional[bytes]:
    binary_packet, db_event_data = None, None
    catcher_vehicle = await get_vehicle_by_ros_id(session, police_ros_id)
    runner_vehicle = await get_vehicle_by_ros_id(session, runner_ros_id)
    if not catcher_vehicle or not runner_vehicle: return None

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
        logging.info(f"--- DB에 이벤트 저장 완료: {event_type} ---")
        return binary_packet
    return None

# --- 메인 서비스 핸들러 ---
async def handle_vehicle_registration(data: bytes) -> Tuple[bytes, Optional[bytes]]:
    logging.debug(f"[RECV /register] raw 데이터: {data.hex()}")
    try:
        if len(data) != 32: raise struct.error("Incorrect packet size")
        msg_type, vehicle_id, vehicle_type_int, car_name_bytes, received_hmac = struct.unpack('<BIB10s16s', data)
        if msg_type != MessageType.REGISTER_REQUEST: raise ValueError("Invalid message type")
        if not hmac.compare_digest(_calculate_hmac(data[:16]), received_hmac): raise ValueError("HMAC validation failed")
        request_data = websocket_schema.VehicleRegistrationRequest(
            vehicle_id=vehicle_id, vehicle_type=vehicle_type_int, car_name=car_name_bytes.decode('utf-8').strip('\x00')
        )
        logging.debug(f"[UNPACK] {request_data}")
    except (struct.error, ValueError) as e:
        logging.warning(f"[FAIL] 등록 정보 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

    async with AsyncSessionMaker() as db_session:
        try:
            if await is_car_name_exists(db_session, request_data.car_name):
                return (_create_ros_error_packet(RosErrorCode.DUPLICATE_NAME), None)
            
            vehicle_type_enum = VehicleTypeEnum.POLICE if request_data.vehicle_type == 0 else VehicleTypeEnum.RUNNER
            new_vehicle_instance = Vehicle(
                vehicle_id=request_data.vehicle_id, vehicle_type=vehicle_type_enum, car_name=request_data.car_name
            )
            if vehicle_type_enum == VehicleTypeEnum.POLICE:
                new_vehicle_instance.police_car = PoliceCar()
            
            await save_vehicle(db_session, vehicle_instance=new_vehicle_instance)
            await db_session.refresh(new_vehicle_instance) # 저장 후 id를 얻기 위해 refresh
            logging.info(f"[DB] 새 차량 등록 성공: id={new_vehicle_instance.id}, name='{new_vehicle_instance.car_name}'")
            
            event_data = websocket_schema.VehicleRegisteredEvent(
                id=new_vehicle_instance.id, vehicle_id=new_vehicle_instance.vehicle_id, vehicle_type=request_data.vehicle_type, car_name=request_data.car_name
            )
        except Exception as e:
            logging.error(f"[FAIL] DB 작업 중 에러: {e}")
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), _create_front_error_packet(FrontErrorCode.DATABASE_ERROR))

    ros_response = struct.pack('<BI', MessageType.REGISTER_SUCCESS, event_data.vehicle_id)
    car_name_padded = event_data.car_name.encode('utf-8').ljust(10, b'\x00')
    event_header = struct.pack('<BIIB10s', MessageType.EVENT_VEHICLE_REGISTERED, event_data.id, event_data.vehicle_id, event_data.vehicle_type, car_name_padded)
    front_event = event_header + _calculate_hmac(event_header)
    logging.debug(f"[SEND /register] ROS 응답: {ros_response.hex()}, FE 이벤트: {front_event.hex()}")
    return (ros_response, front_event)

async def handle_location_update(data: bytes, managers: Dict[str, Any]) -> Optional[bytes]:
    """ ✨ 디버깅 print문이 추가된 최종 추적 버전 """
    print("\n--- [DEBUG] 1. handle_location_update 진입 ---")
    try:
        if len(data) != 28: return None
        ros_vehicle_id, pos_x, pos_y, received_hmac = struct.unpack('<Iff16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:12]), received_hmac): return None
        print(f"--- [DEBUG] 2. 데이터 파싱 성공: vehicle_id={ros_vehicle_id} ---")
    except (struct.error, ValueError):
        return None

    game_event_packet_to_broadcast = None

    print("--- [DEBUG] 3. DB 작업(async with) 시작 ---")
    async with AsyncSessionMaker() as db_session:
        try:
            print("--- [DEBUG] 4. get_vehicle_by_ros_id 호출 ---")
            vehicle = await get_vehicle_by_ros_id(db_session, ros_vehicle_id)
            if not vehicle:
                print(f"--- [DEBUG] 5. 차량 없음(id={ros_vehicle_id}), 종료 ---")
                return None
            print(f"--- [DEBUG] 5. 차량 찾음 (DB id={vehicle.id}) ---")

            print("--- [DEBUG] 6. save_vehicle_location 호출 ---")
            await save_vehicle_location(db_session, vehicle_pk_id=vehicle.id, pos_x=pos_x, pos_y=pos_y)
            print("--- [DEBUG] 7. 위치 저장 성공 ---")

            if vehicle.vehicle_type == VehicleTypeEnum.RUNNER:
                print("--- [DEBUG] 8. 충돌 감지 로직 시작 ---")
                stmt = select(Vehicle).where(Vehicle.vehicle_type == VehicleTypeEnum.POLICE).options(selectinload(Vehicle.locations))
                result = await db_session.execute(stmt)
                police_cars = result.scalars().unique().all()
                print(f"--- [DEBUG] 9. 경찰차 {len(police_cars)}대 조회됨 ---")
                
                for police in police_cars:
                    if police.locations:
                        latest_police_loc = police.locations[-1]
                        distance = math.sqrt((pos_x - latest_police_loc.position_x)**2 + (pos_y - latest_police_loc.position_y)**2)
                        
                        if distance < 1.0:
                            print("--- [DEBUG] 10. 충돌 감지! 이벤트 생성 시도 ---")
                            game_event_packet_to_broadcast = await create_and_save_event(
                                db_session, "capture_success", police.vehicle_id, vehicle.vehicle_id
                            )
                            break
                print("--- [DEBUG] 11. 충돌 감지 로직 종료 ---")

        except Exception as e:
            # 에러가 발생하면 정확한 내용을 출력합니다.
            print(f"\n--- [DEBUG] !!! DB 블록 내부에서 심각한 에러 발생 !!! ---")
            print(f"--- [DEBUG] 에러 타입: {type(e).__name__} ---")
            print(f"--- [DEBUG] 에러 내용: {e} ---")
            traceback.print_exc()
            # 에러가 발생했으므로 트랜잭션은 자동으로 롤백됩니다.
            # 하지만 여기서 다시 예외를 발생시켜 함수가 비정상 종료되었음을 알립니다.
            raise
    
    print("--- [DEBUG] 12. DB 작업(async with) 정상 종료 ---")
    if game_event_packet_to_broadcast:
        await managers['event'].broadcast(game_event_packet_to_broadcast)
        await managers['vehicle'].broadcast(game_event_packet_to_broadcast)
    
    loc_broadcast_header = struct.pack('<BIff', MessageType.POSITION_BROADCAST_2D, ros_vehicle_id, pos_x, pos_y)
    return loc_broadcast_header + _calculate_hmac(loc_broadcast_header)

async def handle_vehicle_status_update(data: bytes, managers: Dict[str, Any]) -> Tuple[Optional[bytes], Optional[bytes]]:
    logging.debug(f"[RECV /status] raw 데이터: {data.hex()}")
    try:
        if len(data) != 24: raise struct.error("Incorrect size")
        msg_type, vehicle_id, fuel, collision, status_int, received_hmac = struct.unpack('<BIBBB16s', data)
        if msg_type != MessageType.STATUS_UPDATE_REQUEST: raise ValueError("Invalid type")
        if not hmac.compare_digest(_calculate_hmac(data[:8]), received_hmac): raise ValueError("HMAC fail")
        logging.debug(f"[UNPACK] vehicle_id={vehicle_id}, fuel={fuel}, collision={collision}, status={status_int}")
    except (struct.error, ValueError) as e:
        logging.warning(f"[FAIL] 상태 정보 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

    game_event_packet_to_broadcast = None
    ros_response, front_event = None, None
    async with AsyncSessionMaker() as db_session:
        vehicle = await get_vehicle_by_ros_id(db_session, vehicle_id)
        if not vehicle or vehicle.vehicle_type != VehicleTypeEnum.POLICE:
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), None)

        status_map = {0: PoliceCarStatusEnum.NORMAL, 1: PoliceCarStatusEnum.HALF_DESTROYED, 2: PoliceCarStatusEnum.COMPLETE_DESTROYED}
        new_status_enum = status_map.get(status_int)
        if new_status_enum is None: return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

        await update_vehicle_status(db_session, vehicle.id, fuel, collision, new_status_enum)
        logging.debug(f"[DB] vehicle(id={vehicle.id}) 상태 정보 저장 성공")
        await db_session.refresh(vehicle.police_car)
        
        if vehicle.police_car.status == PoliceCarStatusEnum.COMPLETE_DESTROYED:
            logging.info(f"[EVENT-TRIGGER] 경찰차(id={vehicle_id}) 파괴됨. 검거 실패 이벤트 발생 시도.")
            runners_result = await db_session.execute(select(Vehicle).where(Vehicle.vehicle_type == VehicleTypeEnum.RUNNER))
            active_runner = runners_result.scalars().first()
            if active_runner:
                game_event_packet_to_broadcast = await create_and_save_event(db_session, "capture_failed", vehicle.vehicle_id, active_runner.vehicle_id)

    if game_event_packet_to_broadcast:
        await managers['event'].broadcast(game_event_packet_to_broadcast)
        await managers['vehicle'].broadcast(game_event_packet_to_broadcast)
        
    event_header = struct.pack('<BIBBB', MessageType.STATE_UPDATE, vehicle_id, collision, status_int, fuel)
    front_event = event_header + _calculate_hmac(event_header)
    logging.debug(f"[SEND /status] 프론트엔드로 브로드캐스트할 패킷: {front_event.hex()}")
    return (ros_response, front_event)