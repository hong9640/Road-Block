from fastapi import HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlmodel import select
from sqlalchemy.orm import selectinload
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

async def get_vehicle_by_id(db: AsyncSession, vehicle_id: int) -> models.Vehicle:
    """
    Retrieves a single vehicle by its business logic ID (vehicle_id).
    """
    statement = (
        select(models.Vehicle)
        .where(models.Vehicle.id == vehicle_id)
        .options(selectinload(models.Vehicle.police_car))
    )
    result = await db.execute(statement)
    vehicle = result.scalar_one_or_none()

    if not vehicle:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Vehicle with id {vehicle_id} not found",
        )
    return vehicle

async def update_vehicle_name(db: AsyncSession, vehicle_id: int, car_name: str) -> models.Vehicle:
    """
    Updates the car_name of a specific vehicle.
    """
    vehicle = await get_vehicle_by_id(db, vehicle_id)
    vehicle.car_name = car_name
    db.add(vehicle)
    await db.commit()
    await db.refresh(vehicle)
    # Refresh the relationship to ensure it's loaded after commit
    await db.refresh(vehicle, attribute_names=['police_car'])
    return vehicle

async def delete_vehicle_by_id(db: AsyncSession, vehicle_id: int) -> None:
    """
    Deletes a vehicle by its business logic ID.
    """
    vehicle = await get_vehicle_by_id(db, vehicle_id)
    await db.delete(vehicle)
    await db.commit()
    return

async def get_all_vehicle_events(db: AsyncSession) -> List[EventResponse]:
    """
    Retrieves all vehicle event logs, denormalizing the data for the response.
    """
    statement = select(models.Event).options(selectinload(models.Event.runner))
    result = await db.execute(statement)
    events = result.scalars().all()

    response_events = []
    for event in events:
        if event.runner:
            response_events.append(
                EventResponse(
                    vehicle_id=event.runner.vehicle_id,
                    car_name=event.runner.car_name,
                    vehicle_type=event.runner.vehicle_type,
                    status=event.status,
                    timestamp=event.created_at,
                )
            )
    return response_events
