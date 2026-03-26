import socket
import threading
import time
import dearpygui.dearpygui as dpg
from collections import deque
import math
import random
import json

# Serial connection
# ---------------------------------------

MOCK_MODE = False  # Set to False when drone is connected

# if not MOCK_MODE:
#     port = sys.argv[1]
#     ser = serial.Serial(port, 115200, timeout=0.1)
SOCKET_PATH = "/tmp/drone_telemetry.sock"


# Telemetry storage
# ---------------------------------------

yaw_data = deque(maxlen=200)
pitch_data = deque(maxlen=200)
roll_data = deque(maxlen=200)
time_data = deque(maxlen=200)

motor_values = [0, 0, 0, 0]
joystick = {"pitch": 0.0, "roll": 0.0, "lift": 0.0, "yaw": 0.0}
battery_level = 0.0
fsm_state = "SafeMode"
p_values = {"yaw": 1.0, "pitch": 1.0, "roll": 1.0}

# Accel & Gyro (x, y, z as i16)
accel = {"x": 0, "y": 0, "z": 0}
gyro = {"x": 0, "y": 0, "z": 0}

# Message log: keep only the most recent entries visible in the GUI
message_log = deque(maxlen=50)
message_log_lock = threading.Lock()

FSM_STATES = [
    "SafeMode",
    "PanicMode",
    "ManualMode",
    "CalibrationMode",
    "YawControlMode",
    "FullControlMode",
    "RawMode",
]

FSM_COLORS = {
    "SafeMode": [100, 100, 255],
    "PanicMode": [255, 60, 60],
    "ManualMode": [100, 220, 100],
    "CalibrationMode": [220, 180, 50],
    "YawControlMode": [80, 200, 200],
    "FullControlMode": [180, 100, 255],
    "RawMode": [200, 200, 200],
}

start_time = time.time()


# Helpers
# ---------------------------------------


def log_message(direction: str, msg: str):
    """direction: 'PC>Drone' or 'Drone>PC'"""
    ts = time.strftime("%H:%M:%S")
    with message_log_lock:
        message_log.append((ts, direction, msg))


def manual_value_to_bar(val: float, axis: str):
    if axis == "lift":
        return max(0.0, min(1.0, val / 1000.0))
    return max(0.0, min(1.0, (val + 1000.0) / 2000.0))


if not MOCK_MODE:
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(SOCKET_PATH)
    sock_file = sock.makefile("r")


# Serial reading thread
# ---------------------------------------


def serial_reader():
    global battery_level, fsm_state, joystick, p_values, accel, gyro

    mock_fsm_index = 0
    mock_fsm_timer = time.time()
    mock_p_timer = time.time()
    mock_msg_timer = time.time()

    while True:
        try:
            if not MOCK_MODE:
                line = sock_file.readline()
                if not line:
                    continue

                t = json.loads(line)  # Data on JSON string

                # print("JSON received:", t)
                if "state" in t and "bat_level" in t:
                    fsm_state = t["state"]
                    battery_level = t["bat_level"] / 100.0  # convert 0–100 to 0.0–1.0
                    log_message(
                        "Drone>PC",
                        f"DroneInfo state={fsm_state} bat={battery_level * 100:.1f}%",
                    )
                    continue

                if "DebugRpms" in t:
                    rpms = t["DebugRpms"]["rpms"]
                    for i in range(4):
                        motor_values[i] = rpms[i]
                    log_message("Drone>PC", f"DebugRpms rpms={rpms}")
                    continue

                if "ManualInput" in t:
                    mi = t["ManualInput"]
                    joystick["lift"] = mi["lift"]
                    joystick["roll"] = mi["roll"]
                    joystick["pitch"] = mi["pitch"]
                    joystick["yaw"] = mi["yaw"]
                    log_message(
                        "PC>Drone",
                        f"ManualInput pitch={mi['pitch']} roll={mi['roll']} lift={mi['lift']} yaw={mi['yaw']}",
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
            else:
                t = time.time() - start_time

                # Mock rates
                yaw_data.append(math.sin(t * 2.0) * 50 + random.uniform(-5, 5))
                pitch_data.append(math.sin(t * 1.5) * 30 + random.uniform(-3, 3))
                roll_data.append(math.cos(t * 1.8) * 25 + random.uniform(-3, 3))
                time_data.append(t)

                # Mock motors
                for i in range(4):
                    motor_values[i] = int(
                        400 + math.sin(t + i) * 200 + random.uniform(-20, 20)
                    )

                # Mock joystick
                joystick["pitch"] = round(math.sin(t * 0.7) * 0.8, 3)
                joystick["roll"] = round(math.cos(t * 0.5) * 0.6, 3)
                joystick["lift"] = round((math.sin(t * 0.3) + 1) / 2, 3)
                joystick["yaw"] = round(math.sin(t * 1.1) * 0.9, 3)

                # Mock battery slowly draining
                battery_level = max(0.0, 1.0 - (t / 300.0))

                # Mock accel (i16 range, gravity mostly on z)
                accel["x"] = int(random.uniform(-500, 500))
                accel["y"] = int(random.uniform(-500, 500))
                accel["z"] = int(8192 + random.uniform(-200, 200))  # ~1g on z

                # Mock gyro (i16 range, small drift)
                gyro["x"] = int(math.sin(t * 0.9) * 300 + random.uniform(-50, 50))
                gyro["y"] = int(math.cos(t * 1.1) * 300 + random.uniform(-50, 50))
                gyro["z"] = int(math.sin(t * 0.7) * 200 + random.uniform(-50, 50))

                # Mock FSM cycling every 5 seconds
                if time.time() - mock_fsm_timer > 5:
                    mock_fsm_index = (mock_fsm_index + 1) % len(FSM_STATES)
                    fsm_state = FSM_STATES[mock_fsm_index]
                    mock_fsm_timer = time.time()
                    log_message("Drone>PC", f"FSM transition > {fsm_state}")

                # Mock p_values slowly drifting
                if time.time() - mock_p_timer > 2:
                    for axis in ["yaw", "pitch", "roll"]:
                        p_values[axis] = round(
                            p_values[axis] + random.uniform(-0.05, 0.05), 3
                        )
                    mock_p_timer = time.time()

                # Mock periodic telemetry message
                if time.time() - mock_msg_timer > 1:
                    mock_msg = (
                        f"accel=({accel['x']},{accel['y']},{accel['z']}) "
                        f"gyro=({gyro['x']},{gyro['y']},{gyro['z']}) "
                        f"bat={battery_level:.2f}"
                    )
                    log_message("Drone>PC", mock_msg)

                    # Also mock a PC→Drone command
                    cmd = (
                        f"ManualInput pitch={joystick['pitch']:.2f} "
                        f"roll={joystick['roll']:.2f} "
                        f"lift={joystick['lift']:.2f} "
                        f"yaw={joystick['yaw']:.2f}"
                    )
                    log_message("PC>Drone", cmd)
                    mock_msg_timer = time.time()

                time.sleep(0.05)

        except Exception as e:
            print(f"Serial error: {e}")


# GUI setup
# ---------------------------------------

dpg.create_context()

with dpg.window(label="Drone Ground Station", tag="main_window", width=900, height=800):
    if MOCK_MODE:
        dpg.add_text("⚠ MOCK MODE - No drone connected", color=[255, 200, 0])

    dpg.add_separator()

    # Two column layout
    with dpg.group(horizontal=True):
        # ---- LEFT COLUMN ----
        with dpg.child_window(width=560, height=940, border=False):
            # --- FSM State ---
            dpg.add_text("FSM State", color=[255, 255, 100])
            with dpg.group(horizontal=True):
                dpg.add_text("Current State:", color=[180, 180, 180])
                dpg.add_text(
                    "SafeMode", tag="fsm_display", color=FSM_COLORS["SafeMode"]
                )

            dpg.add_separator()

            # --- Battery ---
            dpg.add_text("Battery Level", color=[255, 255, 100])
            dpg.add_progress_bar(
                tag="battery_bar", default_value=1.0, width=400, overlay="100%"
            )
            dpg.add_text("100%", tag="battery_text")

            dpg.add_separator()

            # --- P values ---
            dpg.add_text("Controller P-Values", color=[255, 255, 100])
            with dpg.table(
                header_row=True,
                borders_innerH=True,
                borders_outerH=True,
                borders_innerV=True,
                borders_outerV=True,
            ):
                dpg.add_table_column(label="Controller")
                dpg.add_table_column(label="P Value")
                for axis in ["yaw", "pitch", "roll"]:
                    with dpg.table_row():
                        dpg.add_text(f"{axis.capitalize()} P:")
                        dpg.add_text("1.000", tag=f"p_{axis}_display")

            dpg.add_separator()

            # --- Joystick Inputs ---
            dpg.add_text("Joystick Inputs", color=[255, 255, 100])
            with dpg.table(
                header_row=True,
                borders_innerH=True,
                borders_outerH=True,
                borders_innerV=True,
                borders_outerV=True,
            ):
                dpg.add_table_column(label="Axis")
                dpg.add_table_column(label="Value")
                dpg.add_table_column(label="Bar")
                for axis in ["pitch", "roll", "lift", "yaw"]:
                    with dpg.table_row():
                        dpg.add_text(axis.capitalize())
                        dpg.add_text("0.000", tag=f"{axis}_val")
                        dpg.add_progress_bar(
                            tag=f"{axis}_bar", default_value=0.5, width=200
                        )

            dpg.add_separator()

            # --- Motor Outputs ---
            dpg.add_text("Motor Outputs", color=[255, 255, 100])
            for i in range(4):
                with dpg.group(horizontal=True):
                    dpg.add_text(f"M{i}:")
                    dpg.add_progress_bar(tag=f"motor{i}", default_value=0, width=350)
                    dpg.add_text("0", tag=f"motor{i}_val")

            dpg.add_separator()

            # --- Accel & Gyro ---
            dpg.add_text("IMU Data", color=[255, 255, 100])
            with dpg.table(
                header_row=True,
                borders_innerH=True,
                borders_outerH=True,
                borders_innerV=True,
                borders_outerV=True,
            ):
                dpg.add_table_column(label="Sensor")
                dpg.add_table_column(label="X (i16)")
                dpg.add_table_column(label="Y (i16)")
                dpg.add_table_column(label="Z (i16)")

                with dpg.table_row():
                    dpg.add_text("Accel")
                    dpg.add_text("0", tag="accel_x")
                    dpg.add_text("0", tag="accel_y")
                    dpg.add_text("0", tag="accel_z")

                with dpg.table_row():
                    dpg.add_text("Gyro")
                    dpg.add_text("0", tag="gyro_x")
                    dpg.add_text("0", tag="gyro_y")
                    dpg.add_text("0", tag="gyro_z")

        # ---- RIGHT COLUMN ----
        with dpg.child_window(width=580, height=940, border=False):
            # --- Rate Plots ---
            dpg.add_text("Sensor Rates", color=[255, 255, 100])

            for label, tag_x, tag_y, series_tag in [
                ("Yaw Rate", "x_axis_yaw", "y_axis_yaw", "yaw_series"),
                ("Pitch Rate", "x_axis_pitch", "y_axis_pitch", "pitch_series"),
                ("Roll Rate", "x_axis_roll", "y_axis_roll", "roll_series"),
            ]:
                with dpg.plot(label=label, height=180, width=-1):
                    dpg.add_plot_axis(dpg.mvXAxis, label="time", tag=tag_x)
                    dpg.add_plot_axis(dpg.mvYAxis, label="deg/s", tag=tag_y)
                    dpg.add_line_series([], [], parent=tag_y, tag=series_tag)

            dpg.add_separator()

            # --- Message Log ---
            dpg.add_text("Message Log", color=[255, 255, 100])
            with dpg.table(
                tag="msg_table",
                header_row=True,
                borders_innerH=True,
                borders_outerH=True,
                borders_innerV=True,
                borders_outerV=True,
                scrollY=True,
                freeze_rows=1,
                height=280,
            ):
                dpg.add_table_column(
                    label="Time", width_fixed=True, init_width_or_weight=70
                )
                dpg.add_table_column(
                    label="Direction", width_fixed=True, init_width_or_weight=90
                )
                dpg.add_table_column(label="Message")


# Update loop
# ---------------------------------------


def update_gui():
    last_rendered_messages = None

    while True:
        if len(time_data) > 0:
            t_list = list(time_data)
            x_min, x_max = t_list[0], t_list[-1]
            rate_series = {
                "yaw": list(yaw_data),
                "pitch": list(pitch_data),
                "roll": list(roll_data),
            }

            dpg.set_value("yaw_series", [t_list, rate_series["yaw"]])
            dpg.set_value("pitch_series", [t_list, rate_series["pitch"]])
            dpg.set_value("roll_series", [t_list, rate_series["roll"]])

            for axis in ["yaw", "pitch", "roll"]:
                dpg.set_axis_limits(f"x_axis_{axis}", x_min, x_max)
                max_abs_rate = max((abs(v) for v in rate_series[axis]), default=1.0)
                y_limit = max(1.0, max_abs_rate * 1.1)
                dpg.set_axis_limits(f"y_axis_{axis}", -y_limit, y_limit)

        # Motors
        for i in range(4):
            val = motor_values[i]
            dpg.set_value(f"motor{i}", val / 800)
            dpg.set_value(f"motor{i}_val", str(val))

        # Joystick
        for axis in ["pitch", "roll", "lift", "yaw"]:
            val = joystick[axis]
            dpg.set_value(f"{axis}_val", f"{val:.3f}")
            dpg.set_value(f"{axis}_bar", manual_value_to_bar(val, axis))

        # Battery
        dpg.set_value("battery_bar", battery_level)
        dpg.set_value("battery_text", f"{battery_level * 100:.1f}%")
        dpg.configure_item("battery_bar", overlay=f"{battery_level * 100:.1f}%")

        # FSM
        dpg.set_value("fsm_display", fsm_state)
        dpg.configure_item(
            "fsm_display", color=FSM_COLORS.get(fsm_state, [255, 255, 255])
        )

        # P values
        for axis in ["yaw", "pitch", "roll"]:
            dpg.set_value(f"p_{axis}_display", f"{p_values[axis]:.3f}")

        # Accel & Gyro
        for sensor, data in [("accel", accel), ("gyro", gyro)]:
            for axis in ["x", "y", "z"]:
                dpg.set_value(f"{sensor}_{axis}", str(data[axis]))

        # Message log: only rebuild when the visible messages actually change,
        # otherwise the table keeps fighting the user's scroll position.
        with message_log_lock:
            current_messages = tuple(message_log)

        if current_messages != last_rendered_messages:
            dpg.delete_item("msg_table", children_only=True, slot=1)
            for ts, direction, msg in current_messages:
                color = [100, 220, 255] if direction == "Drone>PC" else [220, 180, 100]
                with dpg.table_row(parent="msg_table"):
                    dpg.add_text(ts)
                    dpg.add_text(direction, color=color)
                    dpg.add_text(msg)
            last_rendered_messages = current_messages

        time.sleep(0.05)


# Launch threads
# ---------------------------------------

threading.Thread(target=serial_reader, daemon=True).start()
threading.Thread(target=update_gui, daemon=True).start()


# Start GUI
# ---------------------------------------

dpg.create_viewport(title="Drone Interface", width=920, height=900)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
