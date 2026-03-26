# Serial reading thread
# ---------------------------------------
import json

from ui.main import log_message


def serial_reader():
    global battery_level, fsm_state, joystick, p_values, accel, gyro

    while True:
        try:
            line = sock_file.readline()
            if not line:
                continue

            t = json.loads(line)  # Data on JSON string

            # print("JSON received:", t)
            #
            if "state" in t and "bat_level" in t:
                fsm_state = t["state"]
                battery_level = t["bat_level"] / 100.0  # convert 0–100 to 0.0–1.0
                log_message(
                    "Drone>PC",
                    f"DroneInfo state={fsm_state} bat={battery_level * 100:.1f}%",
                )
                continue

            if "accel_x" in t and "gyro_x" in t:
                yaw_data.append(t.get("yaw", 0.0))
                pitch_data.append(t.get("pitch", 0.0))
                roll_data.append(t.get("roll", 0.0))
                time_data.append(time.time() - start_time)

                motors = t.get("motors", motor_values)
                for i in range(4):
                    motor_values[i] = motors[i]

                accel["x"] = t["accel_x"]
                accel["y"] = t["accel_y"]
                accel["z"] = t["accel_z"]
                gyro["x"] = t["gyro_x"]
                gyro["y"] = t["gyro_y"]
                gyro["z"] = t["gyro_z"]
                p_values["yaw"] = t.get("p_yaw", p_values["yaw"])
                p_values["pitch"] = t.get("p_pitch", p_values["pitch"])
                p_values["roll"] = t.get("p_roll", p_values["roll"])
                log_message(
                    "Drone>PC",
                    (
                        f"Telemetry accel=({accel['x']},{accel['y']},{accel['z']}) "
                        f"gyro=({gyro['x']},{gyro['y']},{gyro['z']}) "
                        f"p=({p_values['yaw']:.1f},{p_values['pitch']:.1f},{p_values['roll']:.1f})"
                    ),
                )

                continue

            yaw_data.append(t["yaw_rate"])
            pitch_data.append(t["pitch_rate"])
            roll_data.append(t["roll_rate"])
            time_data.append(time.time() - start_time)

            for i in range(4):
                motor_values[i] = t["motors"][i]

            joystick["pitch"] = t["pitch"]
            joystick["roll"] = t["roll"]
            joystick["lift"] = t["lift"]
            joystick["yaw"] = t["yaw"]

            battery_level = t["battery"]
            fsm_state = t["fsm_state"]

            p_values["yaw"] = t["p_yaw"]
            p_values["pitch"] = t["p_pitch"]
            p_values["roll"] = t["p_roll"]
        except Exception as e:
            print(f"Serial error: {e}")
