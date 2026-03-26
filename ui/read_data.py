# Serial reading thread
# ---------------------------------------
import json
import socket
import time

import data as stored_data


def log_message(direction: str, msg: str):
    """direction: 'PC>Drone' or 'Drone>PC'"""
    ts = time.strftime("%H:%M:%S")
    stored_data.message_log.append((ts, direction, msg))


def serial_reader():
    global battery_level, fsm_state, joystick, p_values, accel, gyro

    SOCKET_PATH = "/tmp/drone_telemetry.sock"

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(SOCKET_PATH)
    sock_file = sock.makefile("r")

    start_time = time.time()

    while True:
        time.sleep(2)
        try:
            line = sock_file.readline()
            if not line:
                continue

            t = json.loads(line)  # Data on JSON string
            print(t)
        #
        #     fsm_state = t.get("state", "Unknown")
        #     battery_level = t.get("bat_level", 0) / 100.0  # convert 0–100 to 0.0–1.0
        #     log_message(
        #         "Drone>PC",
        #         f"DroneInfo state={fsm_state} bat={battery_level * 100:.1f}%",
        #     )
        #
        #     stored_data.yaw_data.append(t.get("yaw", 0.0))
        #     stored_data.pitch_data.append(t.get("pitch", 0.0))
        #     stored_data.roll_data.append(t.get("roll", 0.0))
        #     stored_data.time_data.append(time.time() - start_time)
        #
        #     motors = t.get("motors", stored_data.motor_values)
        #
        #     for i in range(4):
        #         stored_data.motor_values[i] = motors[i]
        #
        #     stored_data.accel_raw["x"] = t.get("accel_x", 0.0)
        #     stored_data.accel_raw["y"] = t.get("accel_y", 0.0)
        #     stored_data.accel_raw["z"] = t.get("accel_z", 0.0)
        #     stored_data.gyro_raw["x"] = t.get("gyro_x", 0.0)
        #     stored_data.gyro_raw["y"] = t.get("gyro_y", 0.0)
        #     stored_data.gyro_raw["z"] = t.get("gyro_z", 0.0)
        #
        #     log_message(
        #         "Drone>PC",
        #         (
        #             f"Telemetry accel=({stored_data.accel_raw['x']},{stored_data.accel_raw['y']},{stored_data.accel_raw['z']}) "
        #             f"gyro=({stored_data.gyro_raw['x']},{stored_data.gyro_raw['y']},{stored_data.gyro_raw['z']}) "
        #         ),
        #     )
        #
        #     stored_data.joystick["pitch"] = t.get("pitch", 0.0)
        #     stored_data.joystick["roll"] = t.get("roll", 0.0)
        #     stored_data.joystick["lift"] = t.get("lift", 0.0)
        #     stored_data.joystick["yaw"] = t.get("yaw", 0.0)
        #
        except Exception as e:
            print(f"Serial error: {e}")
