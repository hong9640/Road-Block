from pydantic import BaseModel

class VehicleRegistrationRequest(BaseModel):
    vehicle_id: int
    vehicle_type: int
    car_name: str

class VehicleRegisteredEvent(BaseModel):
    id: int
    vehicle_id: int
    vehicle_type: int
    car_name: str

class VehicleLocationUpdateRequest(BaseModel):
    vehicle_id: int
    position_x: float
    position_y: float

class VehicleLocationBroadcast(BaseModel):
    vehicle_id: int
    position_x: float
    position_y: float

class VehicleStatusUpdateRequest(BaseModel):
    vehicle_id: int
    fuel: int
    collision_count: int
    status_enum: int 

class VehicleStatusBroadcast(BaseModel):
    vehicle_id: int
    fuel: int
    collision_count: int
    status_enum: int

class StartTrackingEvent(BaseModel):
    runner_id: int

class CaptureSuccessEvent(BaseModel):
    catcher_id: int
    runner_id: int

class CatchFailedEvent(BaseModel):
    catcher_id: int
    runner_id: int