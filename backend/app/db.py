# app/db.py

import os
import json
from typing import AsyncGenerator, Optional

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlmodel import SQLModel, Field, select
from app.schemas.websocket_schema import VehicleRegistrationRequest
# (수정) models.py에서 정의한 실제 데이터베이스 모델들을 가져옵니다.
from app.models.models import Vehicle, VehicleLocation, PoliceCar, Event
from app.models.enums import PoliceCarStatusEnum, VehicleTypeEnum
from app.schemas.websocket_schema import (
    VehicleRegistrationRequest, 
    VehicleLocationUpdateRequest,
    VehicleStatusUpdateRequest
)


DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_USERNAME = os.getenv("DB_USERNAME")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")
DB_SSL_CONFIG = os.getenv("DB_SSL_CONFIG") # .env 파일에서 SSL 설정을 문자열로 가져옵니다.

if not all([DB_HOST, DB_PORT, DB_USERNAME, DB_PASSWORD, DB_NAME]):
    raise ValueError("App Error: Missing database configuration in .env file.")

DATABASE_URL = f"mysql+asyncmy://{DB_USERNAME}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

# SSL 설정을 처리합니다.
connect_args = {}
if DB_SSL_CONFIG:
    try:
        # DB_SSL_CONFIG 환경 변수(문자열)를 JSON으로 파싱하여 connect_args에 추가합니다.
        ssl_config = json.loads(DB_SSL_CONFIG)
        connect_args["ssl"] = ssl_config
    except json.JSONDecodeError:
        # 단순 "true" 문자열일 경우 True 불리언으로 변환합니다.
        if DB_SSL_CONFIG.lower() == "true":
            connect_args["ssl"] = True
        else:
            # 유효하지 않은 형식의 값일 경우 에러를 발생시킵니다.
            raise ValueError(f"Invalid DB_SSL_CONFIG format: {DB_SSL_CONFIG}")

engine = create_async_engine(
    DATABASE_URL,
    echo=True,
    connect_args=connect_args
)

# SessionMaker를 모듈 레벨에서 한 번만 생성합니다.
AsyncSessionMaker = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)


async def get_session() -> AsyncGenerator[AsyncSession, None]:
    """FastAPI 의존성 주입을 위한 비동기 데이터베이스 세션 생성기"""
    async with AsyncSessionMaker() as session:
        yield session


async def create_db_and_tables():
    """SQLModel 메타데이터를 기반으로 모든 테이블을 비동기적으로 생성합니다."""
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)
    print("--- Database tables created successfully. ---")

async def get_vehicle_by_ros_id(session: AsyncSession, ros_vehicle_id: int) -> Optional[Vehicle]:
    """ROS vehicle_id를 사용하여 Vehicle 테이블에서 해당하는 차량 객체를 찾습니다."""
    statement = select(Vehicle).where(Vehicle.vehicle_id == ros_vehicle_id)
    result = await session.execute(statement)
    return result.scalar_one_or_none()


async def is_car_name_exists(session: AsyncSession, car_name: str) -> bool:
    """주어진 차량 이름이 데이터베이스에 이미 존재하는지 확인합니다."""
    statement = select(Vehicle).where(Vehicle.car_name == car_name)
    result = await session.execute(statement)
    return result.first() is not None

async def save_vehicle(session: AsyncSession, vehicle_data: dict) -> Vehicle:
    """
    ERD에 맞게 수정된 저장 로직.
    Vehicle 생성 후, 반환된 id를 PoliceCar의 PK이자 FK인 vehicle_id에 할당합니다.
    """
    # 1. 전달받은 딕셔너리로 Vehicle 객체 생성
    new_vehicle = Vehicle(**vehicle_data)
    session.add(new_vehicle)
    
    # 2. DB에 임시 반영(flush)하여 new_vehicle.id 값을 할당받음
    await session.flush()

    # 3. 차량 종류가 POLICE일 경우, PoliceCar 객체 생성
    if new_vehicle.vehicle_type == VehicleTypeEnum.POLICE:
        # 🌟 핵심: Vehicle의 PK인 id를 PoliceCar의 PK인 vehicle_id에 명시적으로 전달
        new_police_car = PoliceCar(vehicle_id=new_vehicle.id)
        session.add(new_police_car)

    # 4. 모든 변경사항을 DB에 최종 커밋
    await session.commit()
    await session.refresh(new_vehicle)
    return new_vehicle

async def save_vehicle_location(session: AsyncSession, location_data: VehicleLocationUpdateRequest) -> bool:
    """
    (수정) 새로운 차량 위치 정보를 저장하고, 성공 여부를 반환합니다.
    """
    # 1. ROS ID로 내부 PK 찾기
    vehicle = await get_vehicle_by_ros_id(session, location_data.vehicle_id)
    if not vehicle:
        print(f"Location update failed: Vehicle with ROS ID {location_data.vehicle_id} not found.")
        return False # 차량을 찾지 못함

    # 2. 찾은 내부 PK(vehicle.id)를 사용하여 위치 정보 객체 생성
    new_location = VehicleLocation(
        vehicle_id=vehicle.id,
        position_x=location_data.position_x,
        position_y=location_data.position_y
    )
    session.add(new_location)
    await session.commit()
    return True


async def update_vehicle_status(session: AsyncSession, status_data: VehicleStatusUpdateRequest) -> Optional[PoliceCar]:
    """
    (수정) 차량(PoliceCar)의 상태(충돌 횟수, 상태, 연료)를 업데이트합니다.
    """
    # 1. ROS ID로 내부 PK 찾기
    vehicle = await get_vehicle_by_ros_id(session, status_data.vehicle_id)
    if not vehicle:
        print(f"Status update failed: Vehicle with ROS ID {status_data.vehicle_id} not found.")
        return None # 차량을 찾지 못함

    # 2. 찾은 내부 PK(vehicle.id)를 사용하여 PoliceCar 객체 조회
    statement = select(PoliceCar).where(PoliceCar.vehicle_id == vehicle.id)
    result = await session.execute(statement)
    police_car = result.scalar_one_or_none()

    if not police_car:
        return None

    # 3. 상태 업데이트
    police_car.collision_count = status_data.collision_count
    police_car.fuel = status_data.fuel
    
    status_map = {
        0: PoliceCarStatusEnum.NORMAL,
        1: PoliceCarStatusEnum.HALF_DESTROYED,
        2: PoliceCarStatusEnum.COMPLETE_DESTROYED,
    }
    police_car.status = status_map.get(status_data.status_enum, police_car.status)

    session.add(police_car)
    await session.commit()
    await session.refresh(police_car)
    
    return police_car