# Telemetry storage
# ---------------------------------------

from collections import deque
import threading


class ReadData:
    def __init__(self) -> None:
        self.yaw_data = deque(maxlen=200)
        self.pitch_data = deque(maxlen=200)
        self.roll_data = deque(maxlen=200)

        self.time_data = deque(maxlen=200)

        # motor_values = [0, 0, 0, 0]

        # joystick = {"pitch": 0.0, "roll": 0.0, "lift": 0.0, "yaw": 0.0}

        self.accel_raw = {
            "x": deque(maxlen=200),
            "y": deque(maxlen=200),
            "z": deque(maxlen=200),
        }
        self.gyro_raw = {
            "x": deque(maxlen=200),
            "y": deque(maxlen=200),
            "z": deque(maxlen=200),
        }

        self.pres_data = deque(maxlen=200)

        self.is_paused = False


live_data = ReadData()
logged_data = ReadData()

battery_level = 0.0
fsm_state = "SafeMode"

# Message log: keep only the most recent entries visible in the GUI
pause_logs = False
message_log = deque(maxlen=500)
message_log_lock = threading.Lock()
