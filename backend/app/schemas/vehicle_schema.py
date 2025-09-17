from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from app.models import enums


class PoliceCarDetails(BaseModel):
    fuel: int
    collision_count: int
    status: enums.PoliceCarStatusEnum

class VehicleBase(BaseModel):
    car_name: str = Field(..., max_length=50, example="Police-01")
    vehicle_type: enums.VehicleTypeEnum = Field(..., example=enums.VehicleTypeEnum.POLICE)

class VehicleResponse(VehicleBase):
    id: int = Field(..., description="The business identifier for the vehicle", example=1)
    # PoliceCarDetails는 POLICE 타입 차량에만 존재하므로 Optional
    details: Optional[PoliceCarDetails] = None

    class Config:
        from_attributes = True

class VehicleListResponse(BaseModel):
    vehicles: List[VehicleResponse]


class VehicleUpdate(BaseModel):
    """
    Schema for the PATCH request body to update a vehicle's car name.
    """
    car_name: str = Field(..., max_length=50, example="Police-03")

class EventResponse(BaseModel):
    event_id: int
    catcher_id: Optional[int] = None
    runner_id: int
    status: enums.EventStatus
    created_at: datetime

    class Config:
        from_attributes = True

# 💡 변경점 2: 라우터의 response_model에 맞는 최종 응답 모델 정의
class VehicleEventListResponse(BaseModel):
    events: List[EventResponse]
