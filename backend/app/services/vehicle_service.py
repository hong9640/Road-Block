from fastapi import HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select
from sqlalchemy.orm import selectinload
from typing import List
from datetime import datetime, timezone
from app.models import models
from app.schemas.vehicle_schema import EventResponse

async def get_all_vehicles(db: AsyncSession) -> List[models.Vehicle]:
    statement = select(models.Vehicle).options(selectinload(models.Vehicle.police_car)).where(models.Vehicle.deleted_at == None)
    result = await db.execute(statement)
    return result.scalars().all()

async def get_vehicle_by_id(db: AsyncSession, id: int) -> models.Vehicle:
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
    vehicle = await get_vehicle_by_id(db, id)
    vehicle.car_name = car_name
    db.add(vehicle)
    await db.commit()
    await db.refresh(vehicle)
    await db.refresh(vehicle, attribute_names=['police_car'])
    return vehicle

async def delete_vehicle_by_id(db: AsyncSession, id: int) -> None:
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
    statement = (
        select(models.Event)
        .order_by(models.Event.created_at.desc())
        .options(selectinload(models.Event.runner), selectinload(models.Event.catcher))
    )
    result = await db.execute(statement)
    events = result.scalars().all()


    response_events = []
    for event in events:
        response_events.append(
            EventResponse(
                id=event.event_id, 
                catcher_id=event.catcher_id,
                runner_id=event.runner_id,
                status=event.status,
                created_at=event.created_at
            )
        )
    
    return response_events
