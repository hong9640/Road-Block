from enum import IntEnum

class MessageType(IntEnum):
    """메시지 타입 정의"""
    # Ros <-> Backend
    REGISTER_REQUEST = 0xA0
    REGISTER_SUCCESS = 0xA1
    NACK_ERROR = 0x02

    # Backend -> Frontend
    SYSTEM_ERROR = 0x03
    EVENT_VEHICLE_REGISTERED = 0xA2


class RosErrorCode(IntEnum):
    """ROS(임베디드) 통신 에러 코드 (NACK_ERROR 용)"""
    INVALID_FORMAT = 1
    DUPLICATE_NAME = 2


class FrontErrorCode(IntEnum):
    """프론트엔드 통신 에러 코드 (SYSTEM_ERROR 용)"""
    INTERNAL_SERVER_ERROR = 100
    DATABASE_ERROR = 101
