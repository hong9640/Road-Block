from fastapi import APIRouter, Depends, status, Path
from starlette.responses import Response
from sqlalchemy.ext.asyncio import AsyncSession
# 라우터
# Import schemas, async service functions
from app.schemas import vehicle_schema
from app.services import vehicle_service
# Import get_session from the provided db.py
from app.db import get_session
# Corrected import path for models as seen in db.py
from app.models.models import Vehicle

router = APIRouter(
    prefix="/vehicles",
    tags=["Vehicles"],
)

# --- Helper function for response mapping ---

def map_vehicle_to_response(vehicle: Vehicle) -> vehicle_schema.VehicleResponse:
    response_data = vehicle_schema.VehicleResponse.model_validate(vehicle)
    if vehicle.police_car:
        response_data.details = vehicle.police_car
    return response_data

# --- API Endpoints (all async) ---

@router.get(
    "",
    response_model=vehicle_schema.VehicleListResponse,
    summary="차량 리스트 조회 (Get Vehicle List)"
)
async def list_vehicles(db: AsyncSession = Depends(get_session)):
    vehicles_from_db = await vehicle_service.get_all_vehicles(db)
    response_vehicles = [map_vehicle_to_response(v) for v in vehicles_from_db]
    return {"vehicles": response_vehicles}

# 차량 조회
@router.get(
    "/{vehicle_id}",
    response_model=vehicle_schema.VehicleResponse,
    summary="단일 차량 조회 (Get Single Vehicle)"
)
async def read_vehicle(
    vehicle_id: int = Path(..., title="The business ID of the vehicle to get", ge=1),
    db: AsyncSession = Depends(get_session)
):
    vehicle_from_db = await vehicle_service.get_vehicle_by_id(db, vehicle_id=vehicle_id)
    return map_vehicle_to_response(vehicle_from_db)

@router.patch(
    "/{vehicle_id}",
    response_model=vehicle_schema.VehicleResponse,
    summary="차량 정보 수정 (Update Vehicle Information)"
)
async def patch_vehicle(
    update_data: vehicle_schema.VehicleUpdate,
    vehicle_id: int = Path(..., title="The business ID of the vehicle to update", ge=1),
    db: AsyncSession = Depends(get_session)
):
    updated_vehicle = await vehicle_service.update_vehicle_name(
        db, vehicle_id=vehicle_id, car_name=update_data.car_name
    )
    return map_vehicle_to_response(updated_vehicle)

@router.delete(
    "/{vehicle_id}",
    status_code=status.HTTP_204_NO_CONTENT,
    summary="차량 삭제 (Delete Vehicle)"
)
async def remove_vehicle(
    vehicle_id: int = Path(..., title="The business ID of the vehicle to delete", ge=1),
    db: AsyncSession = Depends(get_session)
):
    await vehicle_service.delete_vehicle_by_id(db, vehicle_id=vehicle_id)
    return Response(status_code=status.HTTP_204_NO_CONTENT)

@router.get(
    "/events",
    response_model=vehicle_schema.VehicleEventListResponse,
    summary="차량 사건 로그 조회 (Get Vehicle Event Logs)"
)
async def list_vehicle_events(db: AsyncSession = Depends(get_session)):
    events_data = await vehicle_service.get_all_vehicle_events(db)
    return {"events": events_data}