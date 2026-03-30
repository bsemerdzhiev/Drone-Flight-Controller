# Telemetry storage
# ---------------------------------------

from collections import deque
import threading

MAX_SIZE = 200
MAX_LOG_QUEUE_SIZE = 500


class ReadData:
    def __init__(self) -> None:
        self.yaw_data = deque(maxlen=MAX_SIZE)
        self.pitch_data = deque(maxlen=MAX_SIZE)
        self.roll_data = deque(maxlen=MAX_SIZE)

        self.yaw_kalman = deque(maxlen=MAX_SIZE)
        self.pitch_kalman = deque(maxlen=MAX_SIZE)
        self.roll_kalman = deque(maxlen=MAX_SIZE)

        self.time_data = deque(maxlen=MAX_SIZE)

        # motor_values = [0, 0, 0, 0]
        self.motors = deque(maxlen=MAX_SIZE)

        self.accel_raw = {
            "x": deque(maxlen=MAX_SIZE),
            "y": deque(maxlen=MAX_SIZE),
            "z": deque(maxlen=MAX_SIZE),
        }
        self.gyro_raw = {
            "x": deque(maxlen=MAX_SIZE),
            "y": deque(maxlen=MAX_SIZE),
            "z": deque(maxlen=MAX_SIZE),
        }
        self.pres_data = deque(maxlen=MAX_SIZE)
        self.pres_data_filtered = deque(maxlen=MAX_SIZE)

        self.is_paused = False

        self.battery_level = deque(maxlen=MAX_SIZE)


joystick = {
    "pitch": deque(maxlen=MAX_SIZE),
    "roll": deque(maxlen=MAX_SIZE),
    "lift": deque(maxlen=MAX_SIZE),
    "yaw": deque(maxlen=MAX_SIZE),
    "yaw_p_trim": deque(maxlen=MAX_SIZE),
    "roll_pitch_p_trim": deque(maxlen=MAX_SIZE),
    "roll_pitch_d_trim": deque(maxlen=MAX_SIZE),
}

live_data = ReadData()
logged_data = ReadData()

fsm_state = "SafeMode"

# Message log: keep only the most recent entries visible in the GUI
pause_logs = False
message_log = deque(maxlen=MAX_LOG_QUEUE_SIZE)
message_log_lock = threading.Lock()
