# app/db.py

import os
import json
from typing import AsyncGenerator, Optional
from sqlalchemy.orm import selectinload
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlmodel import SQLModel, Field, select
from app.schemas.websocket_schema import VehicleRegistrationRequest
# (수정) models.py에서 정의한 실제 데이터베이스 모델들을 가져옵니다.
from app.models import models
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


async def get_vehicle_by_ros_id(session: AsyncSession, ros_id: int) -> Vehicle | None:
    """
    ROS ID(vehicle_id)를 사용하여 특정 Vehicle 객체를 반환합니다.
    이때 연관된 police_car, caught_events, run_events 정보도 함께 '즉시 로딩'합니다.
    """
    statement = (
        select(Vehicle)
        .options(
            selectinload(Vehicle.police_car),
            selectinload(Vehicle.caught_events), 
            selectinload(Vehicle.run_events)
        )
        .where(Vehicle.vehicle_id == ros_id)
    )
    result = await session.execute(statement)
    return result.scalars().first()


async def is_car_name_exists(session: AsyncSession, car_name: str) -> bool:
    """주어진 차량 이름이 데이터베이스에 이미 존재하는지 확인합니다."""
    statement = select(Vehicle).where(Vehicle.car_name == car_name)
    result = await session.execute(statement)
    return result.first() is not None

async def save_vehicle(session: AsyncSession, vehicle_instance: Vehicle) -> Vehicle:
    """
    미리 생성된 Vehicle 모델 인스턴스를 받아 DB에 저장합니다.
    (연관된 PoliceCar 인스턴스가 있다면 함께 저장됩니다.)
    """
    session.add(vehicle_instance)
    await session.commit()
    await session.refresh(vehicle_instance)
    return vehicle_instance

async def save_vehicle_location(session: AsyncSession, vehicle_pk_id: int, pos_x: float, pos_y: float) -> bool:
    """ ✨ (수정됨) 차량의 내부 ID(PK)와 위치 좌표를 받아 DB에 저장합니다. """
    new_location = VehicleLocation(
        vehicle_id=vehicle_pk_id,
        position_x=pos_x,
        position_y=pos_y
    )
    session.add(new_location)
    await session.commit()
    return True


async def update_vehicle_status(session: AsyncSession, vehicle_pk_id: int, fuel: int, collision_count: int, status: PoliceCarStatusEnum) -> Optional[PoliceCar]:
    """ ✨ (수정됨) PoliceCar의 상태를 업데이트합니다. """
    statement = select(PoliceCar).where(PoliceCar.vehicle_id == vehicle_pk_id)
    result = await session.execute(statement)
    police_car = result.scalar_one_or_none()

    if not police_car:
        return None

    police_car.collision_count = collision_count
    police_car.fuel = fuel
    police_car.status = status
    
    session.add(police_car)
    await session.commit()
    await session.refresh(police_car)
    return police_car

async def get_all_vehicles(session: AsyncSession) -> list[Vehicle]:
    """
    (수정됨) 모든 Vehicle 객체 리스트를 관계된 locations, police_car 정보와 함께 반환합니다.
    """
    statement = select(Vehicle).options(
        selectinload(Vehicle.locations),  # 차량 위치 정보들을 함께 로드
        selectinload(Vehicle.police_car)   # 경찰차 상태 정보를 함께 로드
    ).where(models.Vehicle.deleted_at == None)
    result = await session.execute(statement)
    # .unique()를 사용하여 중복된 Vehicle 객체를 제거합니다.
    return result.scalars().unique().all()

async def save_event(session: AsyncSession, event_data: dict) -> Event:
    """ ✨ (수정됨) 딕셔너리로부터 Event 모델을 생성하여 저장합니다. """
    new_event = Event(**event_data)
    session.add(new_event)
    await session.commit()
    await session.refresh(new_event)
    return new_event

async def get_all_events(session: AsyncSession) -> list[Event]:
    """
    데이터베이스에 저장된 모든 Event 객체 리스트를 반환합니다.
    이때 연관된 runner와 catcher 정보도 함께 '즉시 로딩'합니다.
    """
    statement = (
        select(Event)
        .options(selectinload(Event.runner), selectinload(Event.catcher))
        .order_by(Event.event_id)
    )
    result = await session.execute(statement)
    # unique()를 추가하여 중복을 제거하고 관계 데이터를 올바르게 합칩니다.
    return result.scalars().unique().all()