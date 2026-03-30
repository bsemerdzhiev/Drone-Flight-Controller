# Telemetry storage
# ---------------------------------------

from collections import deque
import threading

MAX_SIZE = 200
MAX_LOG_QUEUE_SIZE = 500


def _init_deque(value=0) -> deque:
    return deque([value] * MAX_SIZE, maxlen=MAX_SIZE)


class ReadData:
    def __init__(self) -> None:
        self.yaw_data = _init_deque()
        self.pitch_data = _init_deque()
        self.roll_data = _init_deque()
        self.yaw_kalman = _init_deque()
        self.pitch_kalman = _init_deque()
        self.roll_kalman = _init_deque()
        self.time_data = _init_deque()
        self.motors = deque([[0, 0, 0, 0]] * MAX_SIZE, maxlen=MAX_SIZE)
        self.accel_raw = {
            "x": _init_deque(),
            "y": _init_deque(),
            "z": _init_deque(),
        }
        self.gyro_raw = {
            "x": _init_deque(),
            "y": _init_deque(),
            "z": _init_deque(),
        }
        self.pres_data = _init_deque()
        self.pres_data_filtered = _init_deque()
        self.is_paused = False
        self.battery_level = _init_deque()


joystick = {
    "pitch": _init_deque(),
    "roll": _init_deque(),
    "lift": _init_deque(),
    "yaw": _init_deque(),
    "yaw_p_trim": _init_deque(),
    "roll_pitch_p_trim": _init_deque(),
    "roll_pitch_d_trim": _init_deque(),
}

live_data = ReadData()
logged_data = ReadData()

fsm_state = "SafeMode"
telemetry_data_size = 0

# Message log: keep only the most recent entries visible in the GUI
pause_logs = False
message_log = deque(maxlen=MAX_LOG_QUEUE_SIZE)
message_log_lock = threading.Lock()
