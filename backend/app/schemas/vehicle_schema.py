from pydantic import BaseModel, Field, field_validator 
from typing import List, Optional
from datetime import datetime, timezone
from zoneinfo import ZoneInfo
from app.models import enums
KST = ZoneInfo("Asia/Seoul")

class PoliceCarDetails(BaseModel):
    fuel: int
    collision_count: int
    status: enums.PoliceCarStatusEnum

class VehicleBase(BaseModel):
    car_name: str = Field(..., max_length=50, example="Police-01")
    vehicle_type: enums.VehicleTypeEnum = Field(..., example=enums.VehicleTypeEnum.POLICE)

class VehicleResponse(VehicleBase):
    id: int = Field(..., description="The business identifier for the vehicle", example=1)
    details: Optional[PoliceCarDetails] = None

    class Config:
        from_attributes = True

class VehicleListResponse(BaseModel):
    vehicles: List[VehicleResponse]


class VehicleUpdate(BaseModel):
    car_name: str = Field(..., max_length=50, example="Police-03")

class EventResponse(BaseModel):
    id: int
    catcher_id: Optional[int] = None
    runner_id: int
    status: enums.EventStatus
    created_at: datetime

    @field_validator("created_at", mode="before")
    @classmethod
    def convert_to_kst(cls, v: datetime) -> datetime:
        if isinstance(v, datetime):
            if v.tzinfo is None:
                v = v.replace(tzinfo=timezone.utc)
            return v.astimezone(KST)
        return v

    class Config:
        from_attributes = True

class VehicleEventListResponse(BaseModel):
    events: List[EventResponse]
