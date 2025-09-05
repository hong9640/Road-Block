import enum

class VehicleTypeEnum(str, enum.Enum):
    POLICE = "police"
    RUNNER = "runner"

class PoliceCarStatusEnum(str, enum.Enum):
    NORMAL = "normal"
    HALF_DESTROYED = "half_destroyed"
    COMPLETE_DESTROYED = "complete_destroyed"

class EventStatus(str, enum.Enum):
    RUN = "run"
    CATCH = "catch"