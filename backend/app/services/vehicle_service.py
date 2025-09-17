from fastapi import HTTPException, status
from pydantic import ValidationError
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select
from sqlalchemy.orm import selectinload
from sqlalchemy import desc
import logging
from typing import List
from datetime import datetime, timezone
# Corrected import path as per user request
from app.models import models
from app.schemas.vehicle_schema import EventResponse

async def get_all_vehicles(db: AsyncSession) -> List[models.Vehicle]:
    """
    Retrieves a list of all vehicles with their related police car details.
    """
    statement = select(models.Vehicle).options(selectinload(models.Vehicle.police_car)).where(models.Vehicle.deleted_at == None)
    result = await db.execute(statement)
    return result.scalars().all()

async def get_vehicle_by_id(db: AsyncSession, id: int) -> models.Vehicle:
    """
    Retrieves a single vehicle by its business logic ID (vehicle_id).
    """
    statement = (
        select(models.Vehicle)
        .options(selectinload(models.Vehicle.police_car))
        .where(
            models.Vehicle.id == id,
            models.Vehicle.deleted_at == None
            )
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
    ID를 사용하여 차량을 논리적으로 삭제합니다. (실제로는 deleted_at 필드를 업데이트)
    """
    vehicle = await get_vehicle_by_id(db, id=id)
    if not vehicle:
        # 💡 차량이 없을 경우 예외 처리 (예시)
        raise HTTPException(status_code=404, detail="Vehicle not found")

    # 💡 변경점: db.delete() 대신 deleted_at에 현재 시간을 기록
    vehicle.deleted_at = datetime.now(timezone.utc)
    db.add(vehicle)
    await db.commit()
    await db.refresh(vehicle)

async def get_all_vehicle_events(db: AsyncSession) -> List[EventResponse]:
    """모든 차량 이벤트 로그를 조회합니다."""
    statement = (
        select(models.Event)
        .order_by(models.Event.created_at.desc())
        .options(selectinload(models.Event.runner), selectinload(models.Event.catcher))
    )
    result = await db.execute(statement)
    events = result.scalars().all()

    # 💡 변경점: model_validate 대신 수동으로 리스트를 생성합니다.
    response_events = []
    for event in events:
        response_events.append(
            EventResponse(
                # DB 객체의 'event_id'를 스키마의 'event_id' 필드에 명시적으로 전달
                id=event.event_id, 
                catcher_id=event.catcher_id,
                runner_id=event.runner_id,
                status=event.status,
                created_at=event.created_at
            )
        )
    
    return response_events
