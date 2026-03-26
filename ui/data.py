# Telemetry storage
# ---------------------------------------

from collections import deque
import threading

yaw_data = deque(maxlen=200)
pitch_data = deque(maxlen=200)
roll_data = deque(maxlen=200)
time_data = deque(maxlen=200)

motor_values = [0, 0, 0, 0]

joystick = {"pitch": 0.0, "roll": 0.0, "lift": 0.0, "yaw": 0.0}

battery_level = 0.0

fsm_state = "SafeMode"

# Accel & Gyro (x, y, z as i16)
accel_raw = {"x": 0, "y": 0, "z": 0}
gyro_raw = {"x": 0, "y": 0, "z": 0}

# Message log: keep only the most recent entries visible in the GUI
message_log = deque(maxlen=50)
message_log_lock = threading.Lock()
