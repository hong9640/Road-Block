from fastapi import HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select
from sqlalchemy.orm import selectinload
from sqlalchemy import desc
import logging
from typing import List

# Corrected import path as per user request
from app.models import models
from app.schemas.vehicle_schema import EventResponse

async def get_all_vehicles(db: AsyncSession) -> List[models.Vehicle]:
    """
    Retrieves a list of all vehicles with their related police car details.
    """
    statement = select(models.Vehicle).options(selectinload(models.Vehicle.police_car))
    result = await db.execute(statement)
    return result.scalars().all()

async def get_vehicle_by_id(db: AsyncSession, id: int) -> models.Vehicle:
    """
    Retrieves a single vehicle by its business logic ID (vehicle_id).
    """
    statement = (
        select(models.Vehicle)
        .options(selectinload(models.Vehicle.police_car))
        .where(models.Vehicle.id == id)
    )
    result = await db.execute(statement)
    vehicle = result.scalar_one_or_none()

    if not vehicle:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Vehicle with id {id} not found",
        )
    return vehicle

async def update_vehicle_name(db: AsyncSession, id: int, car_name: str) -> models.Vehicle:
    """
    Updates the car_name of a specific vehicle.
    """
    vehicle = await get_vehicle_by_id(db, id)
    vehicle.car_name = car_name
    db.add(vehicle)
    await db.commit()
    await db.refresh(vehicle)
    # Refresh the relationship to ensure it's loaded after commit
    await db.refresh(vehicle, attribute_names=['police_car'])
    return vehicle

async def delete_vehicle_by_id(db: AsyncSession, id: int) -> None:
    """
    Deletes a vehicle by its business logic ID.
    """
    vehicle = await get_vehicle_by_id(db, id)
    await db.delete(vehicle)
    await db.commit()
    return

async def get_all_vehicle_events(db: AsyncSession) -> List[EventResponse]:
    """모든 차량 이벤트 로그를 조회합니다."""
    statement = (
        select(models.Event)
        .order_by(models.Event.created_at.desc())
        .options(
            selectinload(models.Event.runner),
            selectinload(models.Event.catcher) # catcher 정보도 함께 로딩
        )
    )
    result = await db.execute(statement)
    events = result.scalars().all()
    return [EventResponse.model_validate(event) for event in events]
