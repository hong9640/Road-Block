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
    vehicle_id: int = Field(..., description="The business identifier for the vehicle", example=1)
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
    vehicle_id: int 
    car_name: str
    vehicle_type: enums.VehicleTypeEnum 
    status: enums.EventStatus 
    timestamp: datetime 

    class Config:
        from_attributes = True


class VehicleEventListResponse(BaseModel):
    events: List[EventResponse]
