# app/db.py

import os
import json
from typing import AsyncGenerator, Optional
from sqlalchemy.orm import selectinload
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlmodel import SQLModel, select
from app.models import models
from app.models.models import Vehicle, VehicleLocation, PoliceCar, Event
from app.models.enums import PoliceCarStatusEnum, EventStatus


DB_HOST = os.getenv("DB_HOST")
DB_PORT = os.getenv("DB_PORT")
DB_USERNAME = os.getenv("DB_USERNAME")
DB_PASSWORD = os.getenv("DB_PASSWORD")
DB_NAME = os.getenv("DB_NAME")
DB_SSL_CONFIG = os.getenv("DB_SSL_CONFIG") 

if not all([DB_HOST, DB_PORT, DB_USERNAME, DB_PASSWORD, DB_NAME]):
    raise ValueError("App Error: Missing database configuration in .env file.")

DATABASE_URL = f"mysql+asyncmy://{DB_USERNAME}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

connect_args = {}
if DB_SSL_CONFIG:
    try:
        ssl_config = json.loads(DB_SSL_CONFIG)
        connect_args["ssl"] = ssl_config
    except json.JSONDecodeError:
        if DB_SSL_CONFIG.lower() == "true":
            connect_args["ssl"] = True
        else:
            raise ValueError(f"Invalid DB_SSL_CONFIG format: {DB_SSL_CONFIG}")

engine = create_async_engine(
    DATABASE_URL,
    echo=True,
    connect_args=connect_args
)

AsyncSessionMaker = async_sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)


async def get_session() -> AsyncGenerator[AsyncSession, None]:
    async with AsyncSessionMaker() as session:
        yield session


async def create_db_and_tables():
    async with engine.begin() as conn:
        await conn.run_sync(SQLModel.metadata.create_all)
    print("--- Database tables created successfully. ---")


async def get_vehicle_by_ros_id(session: AsyncSession, ros_id: int) -> Vehicle | None:
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
    statement = select(Vehicle).where(Vehicle.car_name == car_name)
    result = await session.execute(statement)
    return result.first() is not None

async def save_vehicle(session: AsyncSession, vehicle_instance: Vehicle) -> Vehicle:
    session.add(vehicle_instance)
    await session.commit()
    await session.refresh(vehicle_instance)
    return vehicle_instance

async def save_vehicle_location(session: AsyncSession, vehicle_pk_id: int, pos_x: float, pos_y: float) -> bool:
    new_location = VehicleLocation(
        vehicle_id=vehicle_pk_id,
        position_x=pos_x,
        position_y=pos_y
    )
    session.add(new_location)
    await session.commit()
    return True


async def update_vehicle_status(session: AsyncSession, vehicle_pk_id: int, fuel: int, collision_count: int, status: PoliceCarStatusEnum) -> Optional[PoliceCar]:
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
    statement = select(Vehicle).options(
        selectinload(Vehicle.locations),  
        selectinload(Vehicle.police_car)  
    ).where(models.Vehicle.deleted_at == None)
    result = await session.execute(statement)
    return result.scalars().unique().all()

async def save_event(session: AsyncSession, event_data: dict) -> Event:
    new_event = Event(**event_data)
    session.add(new_event)
    await session.commit()
    await session.refresh(new_event)
    return new_event

async def get_all_events(session: AsyncSession) -> list[Event]:
    statement = (
        select(Event)
        .options(selectinload(Event.runner), selectinload(Event.catcher))
        .order_by(Event.event_id)
    )
    result = await session.execute(statement)
    return result.scalars().unique().all()

async def has_run_event_occurred(session: AsyncSession, runner_id: int) -> bool:
    statement = select(Event).where(Event.runner_id == runner_id, Event.status == EventStatus.RUN)
    result = await session.execute(select(statement.exists()))
    return result.scalar_one()