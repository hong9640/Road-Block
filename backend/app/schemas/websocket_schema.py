from pydantic import BaseModel, Field

class VehicleRegistrationRequest(BaseModel):
    """ROS로부터 받은 차량 등록 요청 데이터 모델"""
    vehicle_id: int
    vehicle_type: int
    car_name: str

class VehicleRegisteredEvent(BaseModel):
    """
    DB에 차량 등록 완료 후 생성되는 이벤트 데이터 모델.
    이 정보를 기반으로 프론트엔드 브로드캐스트 패킷(0xA2)이 생성됩니다.
    """
    id: int
    vehicle_id: int
    vehicle_type: int
    car_name: str

class VehicleLocationUpdateRequest(BaseModel):
    """ROS로부터 받은 차량 위치 업데이트 데이터 모델"""
    vehicle_id: int
    position_x: float
    position_y: float

class VehicleLocationBroadcast(BaseModel):
    """
    위치 정보 DB 저장 후 프론트엔드에 브로드캐스트할 이벤트 데이터 모델
    """
    vehicle_id: int
    position_x: float
    position_y: float

class VehicleStatusUpdateRequest(BaseModel):
    """ROS로부터 받은 차량 상태 업데이트 데이터 모델"""
    vehicle_id: int
    collision_count: int
    status_enum: int # 0: NORMAL, 1: HALF_DESTROYED, 2: COMPLETE_DESTROYED
    fuel: int

class VehicleStatusBroadcast(BaseModel):
    """
    상태 정보 DB 저장 후 프론트엔드에 브로드캐스트할 이벤트 데이터 모델
    """
    vehicle_id: int
    collision_count: int
    status_enum: int
    fuel: int
