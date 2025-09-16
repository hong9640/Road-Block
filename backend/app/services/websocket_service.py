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
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# --- 프로젝트 내부 모듈 import ---
from app.common.ws_codes import MessageType, RosErrorCode, FrontErrorCode
from app.schemas import websocket_schema
from app.db import (
    is_car_name_exists, save_vehicle, save_vehicle_location, update_vehicle_status,
    get_vehicle_by_ros_id, save_event, AsyncSessionMaker
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

    # 이벤트 패킷은 임베디드와의 통신이므로 vehicle_id를 사용합니다.
    if event_type == "capture_success":
        event_model = websocket_schema.CaptureSuccessEvent(catcher_id=catcher_vehicle.vehicle_id, runner_id=runner_vehicle.vehicle_id)
        binary_packet = create_capture_success_packet(event_model)
        db_event_data = {"status": EventStatus.CATCH, "catcher_id": catcher_vehicle.id, "runner_id": runner_vehicle.id}
    elif event_type == "capture_failed":
        event_model = websocket_schema.CatchFailedEvent(police_id=catcher_vehicle.vehicle_id, runner_id=runner_vehicle.vehicle_id)
        binary_packet = create_catch_failed_packet(event_model)
        db_event_data = {"status": EventStatus.FAILED, "catcher_id": catcher_vehicle.id, "runner_id": runner_vehicle.id}
    
    if binary_packet and db_event_data:
        await save_event(session, db_event_data)
        logging.info(f"--- DB에 이벤트 저장 완료: {event_type} ---")
        return binary_packet
    return None

# --- 메인 서비스 핸들러 ---
async def handle_vehicle_registration(data: bytes, managers: Dict[str, Any]) -> Tuple[bytes, Optional[bytes]]:
    logging.info(f"[RECV /register] raw 데이터: {data.hex()}")
    try:
        if len(data) != 32: raise struct.error("Incorrect packet size")
        msg_type, vehicle_id, vehicle_type_int, car_name_bytes, received_hmac = struct.unpack('<BIB10s16s', data)
        if msg_type != MessageType.REGISTER_REQUEST: raise ValueError("Invalid message type")
        if not hmac.compare_digest(_calculate_hmac(data[:16]), received_hmac): raise ValueError("HMAC validation failed")
        request_data = websocket_schema.VehicleRegistrationRequest(
            vehicle_id=vehicle_id, vehicle_type=vehicle_type_int, car_name=car_name_bytes.decode('utf-8').strip('\x00')
        )
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
            await db_session.refresh(new_vehicle_instance)
            logging.info(f"[DB] 새 차량 등록 성공: id={new_vehicle_instance.id}, name='{new_vehicle_instance.car_name}'")
            
            if vehicle_type_enum == VehicleTypeEnum.RUNNER:
                logging.info(f"도둑 차량(id={new_vehicle_instance.id}) 등록 감지. 추적 시작 이벤트를 생성/전송합니다.")
                run_event_data = {"status": EventStatus.RUN, "runner_id": new_vehicle_instance.id}
                await save_event(db_session, run_event_data)
                
                # ✨ 여기가 수정된 부분입니다 ✨
                # 임베디드와 통신하는 추적 시작 이벤트는 vehicle_id를 사용합니다.
                start_tracking_model = websocket_schema.StartTrackingEvent(runner_id=new_vehicle_instance.vehicle_id)
                start_tracking_packet = create_start_tracking_packet(start_tracking_model)
                await managers['event'].broadcast(start_tracking_packet)
                logging.info(f"추적 시작 이벤트(0xF0) 브로드캐스트 완료: {start_tracking_packet.hex()}")

            event_data = websocket_schema.VehicleRegisteredEvent(
                id=new_vehicle_instance.id, vehicle_id=new_vehicle_instance.vehicle_id, vehicle_type=request_data.vehicle_type, car_name=request_data.car_name
            )
        except Exception as e:
            logging.error(f"[FAIL] DB 작업 중 에러: {e}")
            traceback.print_exc()
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), _create_front_error_packet(FrontErrorCode.DATABASE_ERROR))

    # ROS에 보내는 응답은 vehicle_id 사용
    ros_response = struct.pack('<BI', MessageType.REGISTER_SUCCESS, event_data.vehicle_id)
    car_name_padded = event_data.car_name.encode('utf-8').ljust(10, b'\x00')
    # 프론트엔드에 보내는 등록 이벤트는 id를 사용 (이전 결정 사항 유지)
    event_header = struct.pack('<BIIB10s', MessageType.EVENT_VEHICLE_REGISTERED, event_data.id, event_data.id, event_data.vehicle_type, car_name_padded)
    front_event = event_header + _calculate_hmac(event_header)
    logging.info(f"[SEND /register] ROS 응답: {ros_response.hex()}, FE 이벤트: {front_event.hex()}")
    return (ros_response, front_event)

async def handle_location_update(data: bytes, managers: Dict[str, Any]) -> Tuple[Optional[bytes], Optional[bytes]]:
    """
    차량 위치 정보를 수신하여 DB에 저장하고,
    모든 클라이언트가 사용할 수 있도록 vehicle_id를 담아 브로드캐스트합니다.
    """
    try:
        if len(data) != 28:
            raise struct.error("Incorrect packet size")
        ros_vehicle_id, pos_x, pos_y, received_hmac = struct.unpack('<Iff16s', data)
        if not hmac.compare_digest(_calculate_hmac(data[:12]), received_hmac):
            raise ValueError("HMAC validation failed")
    except (struct.error, ValueError) as e:
        logging.warning(f"[FAIL] 위치 정보 파싱/검증 실패: {e}")
        return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

    async with AsyncSessionMaker() as db_session:
        try:
            vehicle = await get_vehicle_by_ros_id(db_session, ros_vehicle_id)
            if not vehicle:
                logging.warning(f"요청한 vehicle_id({ros_vehicle_id})를 찾을 수 없습니다.")
                return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), None)

            # DB에 위치 정보 저장
            await save_vehicle_location(db_session, vehicle_pk_id=vehicle.id, pos_x=pos_x, pos_y=pos_y)

        except Exception as e:
            logging.error(f"--- [FAIL] 위치 업데이트 중 DB 에러: {e} ---")
            traceback.print_exc()
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), _create_front_error_packet(FrontErrorCode.DATABASE_ERROR))
    
    # ✨ 최종 로직 ✨
    # 모든 클라이언트(프론트엔드, 임베디드)가 공통으로 사용하는
    # vehicle_id를 담아 브로드캐스트 패킷을 생성합니다.
    loc_broadcast_header = struct.pack(
        '<BIff', 
        MessageType.POSITION_BROADCAST_2D, 
        vehicle.vehicle_id, # 👈 vehicle_id 사용으로 통일
        pos_x, 
        pos_y
    )
    broadcast_event = loc_broadcast_header + _calculate_hmac(loc_broadcast_header)
    
    # 성공 시에는 브로드캐스트할 패킷만 반환합니다.
    return (None, broadcast_event)

async def handle_vehicle_status_update(data: bytes, managers: Dict[str, Any]) -> Tuple[Optional[bytes], Optional[bytes]]:
    try:
        if len(data) != 24: raise struct.error("Incorrect size")
        msg_type, vehicle_id, fuel, collision, status_int, received_hmac = struct.unpack('<BIBBB16s', data)
        if msg_type != MessageType.STATUS_UPDATE_REQUEST: raise ValueError("Invalid type")
        if not hmac.compare_digest(_calculate_hmac(data[:8]), received_hmac): raise ValueError("HMAC fail")
    except (struct.error, ValueError) as e:
        return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)

    game_event_packet_to_broadcast = None
    async with AsyncSessionMaker() as db_session:
        try:
            vehicle = await get_vehicle_by_ros_id(db_session, vehicle_id)
            if not vehicle or vehicle.vehicle_type != VehicleTypeEnum.POLICE:
                return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), None)
            
            status_map = {s.value: i for i, s in enumerate(PoliceCarStatusEnum)}
            new_status_enum = next((k for k,v in status_map.items() if v == status_int), None)

            if new_status_enum is None: return (_create_ros_error_packet(RosErrorCode.INVALID_FORMAT), None)
            await update_vehicle_status(db_session, vehicle.id, fuel, collision, new_status_enum)
            await db_session.refresh(vehicle.police_car)
            if vehicle.police_car.status == PoliceCarStatusEnum.COMPLETE_DESTROYED:
                runners_result = await db_session.execute(select(Vehicle).where(Vehicle.vehicle_type == VehicleTypeEnum.RUNNER, Vehicle.run_events.any(Event.status == EventStatus.RUN)))
                active_runner = runners_result.scalars().first()
                if active_runner:
                    game_event_packet_to_broadcast = await create_and_save_event(db_session, "capture_failed", vehicle.vehicle_id, active_runner.vehicle_id)
        except Exception as e:
            logging.error(f"--- [FAIL] 상태 업데이트 중 DB 에러: {e} ---")
            traceback.print_exc()
            return (_create_ros_error_packet(RosErrorCode.INVALID_DATA), _create_front_error_packet(FrontErrorCode.DATABASE_ERROR))

    if game_event_packet_to_broadcast:
        await managers['event'].broadcast(game_event_packet_to_broadcast)
        await managers['vehicle'].broadcast(game_event_packet_to_broadcast)
        
    event_header = struct.pack('<BIBBB', MessageType.STATE_UPDATE, vehicle.id, collision, status_int, fuel)
    front_event = event_header + _calculate_hmac(event_header)
    return (None, front_event)

async def handle_incoming_event(data: bytes, managers: Dict[str, Any]) -> Optional[bytes]:
    try:
        message_type = data[0]
        packet_to_broadcast = None
        async with AsyncSessionMaker() as db_session:
            if message_type == MessageType.EVENT_CATCH:
                if len(data) != 25: raise struct.error("Incorrect packet size")
                _, catcher_id, runner_id, received_hmac = struct.unpack('<BII16s', data)
                if not hmac.compare_digest(_calculate_hmac(data[:9]), received_hmac): raise ValueError("HMAC validation failed")
                logging.info(f"[RECV /event] 검거 성공. catcher={catcher_id}, runner={runner_id}")
                packet_to_broadcast = await create_and_save_event(db_session, "capture_success", catcher_id, runner_id)
            elif message_type == MessageType.EVENT_CATCH_FAILED:
                if len(data) != 25: raise struct.error("Incorrect packet size")
                _, police_id, runner_id, received_hmac = struct.unpack('<BII16s', data)
                if not hmac.compare_digest(_calculate_hmac(data[:9]), received_hmac): raise ValueError("HMAC validation failed")
                logging.info(f"[RECV /event] 검거 실패. police={police_id}, runner={runner_id}")
                packet_to_broadcast = await create_and_save_event(db_session, "capture_failed", police_id, runner_id)
            else:
                raise ValueError("Invalid event message type")

        if packet_to_broadcast:
            await managers['event'].broadcast(packet_to_broadcast)
            await managers['vehicle'].broadcast(packet_to_broadcast)

    except (struct.error, ValueError) as e:
        logging.warning(f"[FAIL] 이벤트 정보 파싱/검증 실패: {e}")
        return _create_ros_error_packet(RosErrorCode.INVALID_FORMAT)
    except Exception as e:
        logging.error(f"이벤트 메시지 처리 중 에러 발생: {e}")
        traceback.print_exc()
        return _create_ros_error_packet(RosErrorCode.INVALID_DATA)
    
    return None