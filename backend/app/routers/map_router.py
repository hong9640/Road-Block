from fastapi import APIRouter, Depends, status
from sqlalchemy.ext.asyncio import AsyncSession

from app.db import get_session

router = APIRouter(prefix="/maps", tags=["지도"])

@router.get("/{mad_id}")
async def get_maps(session: AsyncSession = Depends(get_session)):
    pass