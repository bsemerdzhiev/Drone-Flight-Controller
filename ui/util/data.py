# Telemetry storage
# ---------------------------------------

from collections import deque
import threading
import numpy as np

MAX_SIZE_LIVE = 200
MAX_SIZE_LOG = 10_000
MAX_LOG_QUEUE_SIZE = 500


def _init_deque(size=MAX_SIZE_LIVE, value=0) -> deque:
    return deque([value] * size, maxlen=size)


def _init_deque_array(size=MAX_SIZE_LIVE, value=[0, 0, 0]) -> deque:
    return deque([value] * size, maxlen=size)


class ReadData:
    def __init__(self, size) -> None:
        self.yaw_data = _init_deque(size)
        self.pitch_data = _init_deque(size)
        self.roll_data = _init_deque(size)
        self.yaw_kalman = _init_deque(size)
        self.pitch_kalman = _init_deque(size)
        self.roll_kalman = _init_deque(size)
        self.time_data = _init_deque(size)
        self.motors = [
            _init_deque(size),
            _init_deque(size),
            _init_deque(size),
            _init_deque(size),
        ]
        self.accel_raw = {
            "x": _init_deque(size),
            "y": _init_deque(size),
            "z": _init_deque(size),
        }
        self.gyro_raw = {
            "x": _init_deque(size),
            "y": _init_deque(size),
            "z": _init_deque(size),
        }
        self.pres_data = _init_deque(size)
        self.pres_data_filtered = _init_deque(size)
        self.battery_level = _init_deque(size)

        self.general_data = {
            "time_for_main_loop": _init_deque(size),
        }

        self.pid_info = {
            "selected_height": _init_deque(size),
        }
        self.calibration_data = {
            "averaged_accel": _init_deque_array(size),
            "averaged_gyro": _init_deque_array(size),
            "averaged_ypr": _init_deque_array(size),
        }


joystick = {
    "pitch": _init_deque(),
    "roll": _init_deque(),
    "lift": _init_deque(),
    "yaw": _init_deque(),
    "yaw_p_trim": _init_deque(),
    "roll_pitch_p_trim": _init_deque(),
    "roll_pitch_d_trim": _init_deque(),
}

bluetooth = {
    "rssi": 0.0,
    "com_mode": False,
}

start_time = None

pause_logs = False

received_packages = _init_deque()
time_between_main_loop_runs = _init_deque()


baro_var_calc = np.array([0.0])
accel_var_calc = np.array([0.0])

live_data = ReadData(MAX_SIZE_LIVE)
logged_data = ReadData(MAX_SIZE_LOG)

chosen_sensors = False
sensor_names = ["DMP", "Kalman(Simplified)"]

fsm_state = "SafeMode"
telemetry_data_size = 0

# Message log: keep only the most recent entries visible in the GUI
message_log = deque(maxlen=MAX_LOG_QUEUE_SIZE)
message_log_lock = threading.Lock()
