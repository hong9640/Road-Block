from sqlmodel import Field, Relationship, SQLModel, text
from sqlalchemy import Column, Enum as SQLAlchemyEnum
from datetime import datetime
from typing import List, Optional
from . import enums

# --- 테이블 모델 정의 ---

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
    vehicle_id: int = Field(foreign_key="vehicle.id")
    position_x: float
    position_y: float
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )
    # Relationship
    vehicle: "Vehicle" = Relationship(back_populates="locations")

class PoliceCar(SQLModel, table=True):
    vehicle_id: Optional[int] = Field(default=None, foreign_key="vehicle.id", primary_key=True)
    fuel: int = Field(default=100, nullable=False)
    collision_count: int = Field(default=0, nullable=False)
    status: enums.PoliceCarStatusEnum = Field(
        sa_column=Column(SQLAlchemyEnum(enums.PoliceCarStatusEnum),
        default=enums.PoliceCarStatusEnum.NORMAL,
        nullable=False)
    )
    vehicle: "Vehicle" = Relationship(back_populates="police_car")

class Event(SQLModel, table=True):
    event_id: Optional[int] = Field(default=None, primary_key=True)
    catcher_id: Optional[int] = Field(default=None, foreign_key="vehicle.id")
    runner_id: int = Field(foreign_key="vehicle.id")
    status: enums.EventStatus = Field(
        sa_column=Column(SQLAlchemyEnum(enums.EventStatus), nullable=False)
    )
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )

    catcher: Optional["Vehicle"] = Relationship(
        back_populates="caught_events",
        sa_relationship_kwargs={"foreign_keys": "[Event.catcher_id]"}
    )
    runner: "Vehicle" = Relationship(
        back_populates="run_events",
        sa_relationship_kwargs={"foreign_keys": "[Event.runner_id]"}
    )


class Vehicle(SQLModel, table=True):
    id: Optional[int] = Field(default=None, primary_key=True)
    vehicle_id: int = Field(unique=True, index=True, nullable=False)
    car_name: str = Field(max_length=50, unique=True, index=True)
    vehicle_type: enums.VehicleTypeEnum = Field(
        sa_column=Column(SQLAlchemyEnum(enums.VehicleTypeEnum),
        nullable=False)
    )
    created_at: datetime = Field(
        default=None,
        sa_column_kwargs={"server_default": text("now()")},
        nullable=False
    )
    deleted_at: Optional[datetime] = Field(default=None)
    
    # --- Relationships ---
    locations: List["VehicleLocation"] = Relationship(back_populates="vehicle")
    police_car: Optional["PoliceCar"] = Relationship(back_populates="vehicle")
    caught_events: List["Event"] = Relationship(
        back_populates="catcher",
        sa_relationship_kwargs={"foreign_keys": "[Event.catcher_id]"}
    )
    run_events: List["Event"] = Relationship(
        back_populates="runner",
        sa_relationship_kwargs={"foreign_keys": "[Event.runner_id]"}
    )