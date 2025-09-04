from fastapi import APIRouter, Depends, status
from sqlalchemy.ext.asyncio import AsyncSession

from app.db import get_session

router = APIRouter(prefix="/vehicles", tags=["차량"])

@router.get("/")
async def get_vehicles(session: AsyncSession = Depends(get_session)):
    pass

@router.get("/{vehicle_id}")
async def get_vehicle(vehicle_id: int, session: AsyncSession = Depends(get_session)):
    pass

@router.patch("/{vehicle_id}")
async def create_vehicle(session: AsyncSession = Depends(get_session)):
    pass

@router.delete("/{vehicle_id}")
async def delete_vehicle(session: AsyncSession = Depends(get_session)):
    pass
