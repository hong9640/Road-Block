from enum import IntEnum

class MessageType(IntEnum):
    """메시지 타입 정의"""
    # Ros <-> Backend
    REGISTER_REQUEST = 0xA0
    POSITION_BROADCAST = 0x13
    STATUS_UPDATE_REQUEST = 0x12
    EVENT_RUN = 0xFF
    # Backend <-> Frontend
    EVENT_VEHICLE_REGISTERED = 0xA2
    POSITION_BROADCAST_2D = 0x11
    STATE_UPDATE = 0x10
    EVENT_TRACE_START = 0xF0
    # 아래 2개는 font, Ros 전부 씀
    EVENT_CATCH_FAILED = 0xFD
    EVENT_CATCH = 0xFE


class ErrorMessageType(IntEnum):
    """에러 메시지 타입 정의"""
    # Ros
    NACK_ERROR = 0x02
    # 프론트
    SYSTEM_ERROR = 0x03

class ErrorCode(IntEnum):
    # 프론트, Ros
    INTERNAL_SERVER_ERROR = 100
    DATABASE_ERROR = 101
    # Ros
    INVALID_FORMAT = 1
    DUPLICATE_NAME = 2
    INVALID_COORDINATE = 3
    INVALID_DATA = 4
