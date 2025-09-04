from pydantic import BaseModel, Field

class VehicleRegistrationRequest(BaseModel):
    """ROS로부터 받은 차량 등록 요청 데이터 모델"""
    vehicle_type: int
    car_name: str

class VehicleRegisteredEvent(BaseModel):
    """
    DB에 차량 등록 완료 후 생성되는 이벤트 데이터 모델.
    이 정보를 기반으로 프론트엔드 브로드캐스트 패킷(0xA2)이 생성됩니다.
    """
    vehicle_id: int
    vehicle_type: int
    car_name: str
