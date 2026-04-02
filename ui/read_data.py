# Serial reading thread
# ---------------------------------------
import json
import socket
import time
import math
import numpy as np

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
        try:
            line = sock_file.readline()
            if not line:
                continue

            # print(line)
            t = json.loads(line)  # Data on JSON string
            # print(t)
            # print("\n\r")

            with stored_data.message_log_lock:
                if "Telemetry" in t:
                    t = t["Telemetry"]

                    # log_message(
                    #     "Drone>PC",
                    #     f"Telemetry dt={t['dt']} state={t['cur_state']} bat={t['bat']} "
                    #     f"motors=[{t['motors'][0]},{t['motors'][1]},{t['motors'][2]},{t['motors'][3]}] "
                    #     f"yaw={t['yaw']:.3f} pitch={t['pitch']:.3f} roll={t['roll']:.3f} "
                    #     f"yaw_kalman={t['yaw_kalman']:.3f} pitch_kalman={t['pitch_kalman']:.3f} roll_kalman={t['roll_kalman']:.3f} "
                    #     f"accel=({t['accel_x']},{t['accel_y']},{t['accel_z']}) "
                    #     f"gyro=({t['gyro_x']},{t['gyro_y']},{t['gyro_z']}) "
                    #     f"battery=({t['bat']}) "
                    #     f"pres={t['pres']} flash={t['logged_in_flash']}",
                    # )
                    # print(t, "\r")

                    if "GeneralData" in t:
                        t = t["GeneralData"]

                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                            fsm_state = t.get("cur_state", "Unknown")

                            stored_data.fsm_state = fsm_state

                        # Time
                        to_add_to.time_data.append(time.time() - start_time)

                        to_add_to.general_data["time_for_main_loop"].append(
                            t["time_for_main_loop"] / 1000
                        )

                        # Battery Raw
                        to_add_to.battery_level.append(t.get("bat", 0))
                        log_message(
                            "Drone>PC",
                            f"[General] state={t.get('cur_state')} bat={t.get('bat')} time_for_main_loop={t.get('time_for_main_loop')}s, dt={t.get('dt')}s flash={t.get('logged_in_flash')}",
                        )

                    if "MotorData" in t:
                        t = t["MotorData"]

                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                        # Motors
                        motor_read = t.get("motors")

                        for i in range(4):
                            to_add_to.motors[i].append(motor_read[i])
                        log_message(
                            "Drone>PC",
                            f"[Motors] M0={t['motors'][0]} M1={t['motors'][1]} M2={t['motors'][2]} M3={t['motors'][3]} flash={t.get('logged_in_flash')}",
                        )

                    if "PositionData" in t:
                        t = t["PositionData"]

                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                        log_message(
                            "Drone>PC",
                            f"[Position] yaw={math.degrees(t.get('yaw', 0.0)):.1f}° pitch={math.degrees(t.get('pitch', 0.0)):.1f}° roll={math.degrees(t.get('roll', 0.0)):.1f}° | kalman yaw={t.get('yaw_kalman', 0):.1f} pitch={t.get('pitch_kalman', 0):.1f} roll={t.get('roll_kalman', 0):.1f} flash={t.get('logged_in_flash')}",
                        )

                        # DMP Data
                        to_add_to.yaw_data.append(math.degrees(t.get("yaw", 0.0)))
                        to_add_to.pitch_data.append(math.degrees(t.get("pitch", 0.0)))
                        to_add_to.roll_data.append(math.degrees(t.get("roll", 0.0)))

                        # Kalman data
                        to_add_to.yaw_kalman.append(t.get("yaw_kalman", 0.0))
                        to_add_to.pitch_kalman.append(t.get("pitch_kalman", 0.0))
                        to_add_to.roll_kalman.append(t.get("roll_kalman", 0.0))

                    if "RawData" in t:
                        t = t["RawData"]

                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                        log_message(
                            "Drone>PC",
                            f"[Raw] accel=({t.get('accel_x')},{t.get('accel_y')},{t.get('accel_z')}) gyro=({t.get('gyro_x')},{t.get('gyro_y')},{t.get('gyro_z')}) flash={t.get('logged_in_flash')}",
                        )

                        # Accellerometer Raw
                        to_add_to.accel_raw["x"].append(t.get("accel_x", 0.0) / 16384)
                        to_add_to.accel_raw["y"].append(t.get("accel_y", 0.0) / 16384)
                        to_add_to.accel_raw["z"].append(t.get("accel_z", 0.0) / 16384)

                        if (
                            abs(stored_data.accel_var_calc[-1] - t.get("accel_z"))
                            > 1e-6
                        ):
                            stored_data.accel_var_calc = np.append(
                                stored_data.accel_var_calc,
                                t.get("accel_z", 0.0) / 16384 - 1.0,
                            )

                        print(
                            "Accel variance", stored_data.accel_var_calc.var(), "\n\r"
                        )

                        # Gyro Raw
                        to_add_to.gyro_raw["x"].append(t.get("gyro_x", 0.0))
                        to_add_to.gyro_raw["y"].append(t.get("gyro_y", 0.0))
                        to_add_to.gyro_raw["z"].append(t.get("gyro_z", 0.0))

                    if "PressureData" in t:
                        t = t["PressureData"]

                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                            if (
                                abs(stored_data.baro_var_calc[-1] - t.get("pres"))
                                > 1e-6
                            ):
                                stored_data.baro_var_calc = np.append(
                                    stored_data.baro_var_calc, t.get("pres")
                                )

                            print(
                                "Baro variance", stored_data.baro_var_calc.var(), "\n\r"
                            )

                        # Pressure Raw
                        to_add_to.pres_data.append(t.get("pres", 0.0))
                        to_add_to.pres_data_filtered.append(
                            t.get("pressure_filtered", 0.0)
                        )

                        log_message(
                            "Drone>PC",
                            f"[Pressure] raw={t.get('pres', 0.0):.2f} filtered={t.get('pressure_filtered', 0.0):.2f} flash={t.get('logged_in_flash')}",
                        )
                    if "PIDInfo" in t:
                        t = t["PIDInfo"]

                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                        # Pressure Raw
                        to_add_to.pid_info["selected_height"].append(
                            t.get("selected_height")
                        )

                        log_message(
                            "Drone>PC",
                            f"[PIDInfo] selected_height={t.get('selected_height', 0.0):.2f} flash={t.get('logged_in_flash')}",
                        )
                    if "CalibrationInfo" in t:
                        t = t["CalibrationInfo"]

                        # print(t, "\n\r")
                        to_add_to = stored_data.logged_data

                        if not t["logged_in_flash"]:
                            to_add_to = stored_data.live_data

                        # Pressure Raw
                        for chosen in [
                            "averaged_accel",
                            "averaged_gyro",
                            "averaged_ypr",
                        ]:
                            to_add_to.calibration_data[chosen].append(t.get(chosen))

                        log_message(
                            "Drone>PC",
                            f"[CalibrationInfo] accel={t.get('averaged_accel')}, gyro={t.get('averaged_gyro')}, ypr={t.get('averaged_ypr')} flash={t.get('logged_in_flash')}",
                        )

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
                    stored_data.telemetry_data_size = t["telemetry_data_size"]

        except Exception as e:
            print(f"Serial error: {e}")
