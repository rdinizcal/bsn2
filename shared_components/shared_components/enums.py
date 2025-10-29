import enum

class Task(enum.Enum):
    INIT = "init"
    NORMAL = "normal"
    CONFIGURE = "configuration"
    ACTIVATE = "activate"
    COLLECT = "collect"
    PROCESS = "process"
    TRANSFER = "transfer"
    RECHARGING = "recharging"
    DEACTIVATE = "deactivate"
    CLEANUP = "cleanup"
    FUSE = "fuse"
    EMIT = "emit_emergency"

class StatusContent(enum.Enum):
    SUCCESS = "success"
    FAIL = "fail"
    RUNNING = "running"
    IDLE = "idle"

class EventType(enum.Enum):
    ACTIVATE = "activate"
    DEACTIVATE = "deactivate"
    RECHARGE = "recharge"
    RECHARGE_COMPLETE = "recharge_complete"