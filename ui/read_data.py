# Serial reading thread
# ---------------------------------------
import json
import socket
import time
import math

import data as stored_data


def log_message(direction: str, msg: str):
    """direction: 'PC>Drone' or 'Drone>PC'"""
    ts = time.strftime("%H:%M:%S")
    with stored_data.message_log_lock:
        stored_data.message_log.append((ts, direction, msg))


def serial_reader():
    global battery_level, fsm_state, joystick, p_values, accel, gyro

    SOCKET_PATH = "/tmp/drone_telemetry.sock"

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(SOCKET_PATH)
    sock_file = sock.makefile("r")

    start_time = time.time()

    while True:
        try:
            line = sock_file.readline()
            if not line:
                continue

            # print(line)
            t = json.loads(line)  # Data on JSON string
            # print(t)
            # print("\n\r")

            if "Telemetry" in t:
                t = t["Telemetry"]

                log_message(
                    "Drone>PC",
                    f"Telemetry dt={t['dt']} state={t['cur_state']} bat={t['bat']} "
                    f"motors=[{t['motors'][0]},{t['motors'][1]},{t['motors'][2]},{t['motors'][3]}] "
                    f"yaw={t['yaw']:.3f} pitch={t['pitch']:.3f} roll={t['roll']:.3f} "
                    f"accel=({t['accel_x']},{t['accel_y']},{t['accel_z']}) "
                    f"gyro=({t['gyro_x']},{t['gyro_y']},{t['gyro_z']}) "
                    f"battery=({t['bat']}) "
                    f"pres={t['pres']} flash={t['logged_in_flash']}",
                )

                to_add_to = stored_data.logged_data

                if not t["logged_in_flash"]:
                    to_add_to = stored_data.live_data

                    fsm_state = t.get("cur_state", "Unknown")

                    stored_data.fsm_state = fsm_state

                to_add_to.yaw_data.append(math.degrees(t.get("yaw", 0.0)))
                to_add_to.pitch_data.append(math.degrees(t.get("pitch", 0.0)))
                to_add_to.roll_data.append(math.degrees(t.get("roll", 0.0)))
                to_add_to.time_data.append(time.time() - start_time)

                to_add_to.motors.append(t.get("motors", [0, 0, 0, 0]))

                to_add_to.accel_raw["x"].append(t.get("accel_x", 0.0))
                to_add_to.accel_raw["y"].append(t.get("accel_y", 0.0))
                to_add_to.accel_raw["z"].append(t.get("accel_z", 0.0))

                to_add_to.gyro_raw["x"].append(t.get("gyro_x", 0.0))
                to_add_to.gyro_raw["y"].append(t.get("gyro_y", 0.0))
                to_add_to.gyro_raw["z"].append(t.get("gyro_z", 0.0))

                to_add_to.gyro_raw["x"].append(t.get("gyro_x", 0.0))
                to_add_to.gyro_raw["y"].append(t.get("gyro_y", 0.0))
                to_add_to.gyro_raw["z"].append(t.get("gyro_z", 0.0))

                to_add_to.pres_data.append(t.get("pres", 0.0))
                to_add_to.battery_level.append(t.get("bat", 0))
            if "ManualInput" in t:
                t = t["ManualInput"]

                log_message(
                    "PC>Drone",
                    f"ManualInput lift={t['lift']:.3f} roll={t['roll']:.3f} "
                    f"pitch={t['pitch']:.3f} yaw={t['yaw']:.3f}",
                )

                for space_pos in [
                    "pitch",
                    "yaw",
                    "roll",
                    "lift",
                    "yaw_p_trim",
                    "roll_pitch_p_trim",
                    "roll_pitch_d_trim",
                ]:
                    stored_data.joystick[space_pos].append(t[space_pos])

        except Exception as e:
            print(f"Serial error: {e}")
