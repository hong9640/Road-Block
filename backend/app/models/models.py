from sqlmodel import Field, Relationship, SQLModel, text
from sqlalchemy import Column, Enum as SQLAlchemyEnum
from datetime import datetime
from typing import List, Optional
from . import enums

class Map(SQLModel, table=True):
    map_id: Optional[int] = Field(default=None, primary_key=True)
    map_name: str = Field(max_length=100, unique=True, index=True) 
    description: Optional[str] = Field(default=None)
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )
    

class VehicleLocation(SQLModel, table=True):
    location_id: Optional[int] = Field(default=None, primary_key=True)
    vehicle_id: int = Field(foreign_key="vehicle.vehicle_id")
    position_x: float
    position_y: float
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )
    vehicle: "Vehicle" = Relationship(back_populates="locations")


class PoliceCar(SQLModel, table=True):
    police_id: Optional[int] = Field(default=None, foreign_key="vehicle.vehicle_id", primary_key=True)
    fuel: int = Field(default=100, nullable=False)
    collision_count: int = Field(default=0, nullable=False)
    
    # enums.py에서 가져온 PoliceCarStatusEnum을 사용합니다.
    status: enums.PoliceCarStatusEnum = Field(
        sa_column=Column(SQLAlchemyEnum(enums.PoliceCarStatusEnum),
        default=enums.PoliceCarStatusEnum.NORMAL,
        nullable=False)
    )
    
    vehicle: "Vehicle" = Relationship(back_populates="police_car")


class Event(SQLModel, table=True):
    event_id: Optional[int] = Field(default=None, primary_key=True)
    runner_id: int = Field(foreign_key="vehicle.vehicle_id", nullable=False)
    
    # enums.py에서 가져온 RunnerCarStatus를 사용합니다.
    status: enums.RunnerCarStatus = Field(
        sa_column=Column(SQLAlchemyEnum(enums.RunnerCarStatus),
        default=enums.RunnerCarStatus.RUN,
        nullable=False)
    )
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )
    vehicle: "Vehicle" = Relationship(back_populates="event")


class Vehicle(SQLModel, table=True):
    
    vehicle_id: Optional[int] = Field(default=None, primary_key=True)
    car_name: str = Field(max_length=50, unique=True, index=True)
    
    # enums.py에서 가져온 VehicleTypeEnum을 사용합니다.
    vehicle_type: enums.VehicleTypeEnum = Field(
        sa_column=Column(SQLAlchemyEnum(enums.VehicleTypeEnum),
        nullable=False)
    )
    
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )
    
    # Relationships
    locations: List["VehicleLocation"] = Relationship(back_populates="vehicle")
    police_car: Optional[PoliceCar] = Relationship(back_populates="vehicle")
    events: List["Event"] = Relationship(back_populates="vehicle")
