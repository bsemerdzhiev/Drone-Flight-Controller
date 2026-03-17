import sys
import serial
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

yaw_data  = deque(maxlen=200)
pitch_data = deque(maxlen=200)
roll_data  = deque(maxlen=200)
time_data  = deque(maxlen=200)

motor_values = [0, 0, 0, 0]
joystick = {"pitch": 0.0, "roll": 0.0, "lift": 0.0, "yaw": 0.0}
battery_level = 0.0
fsm_state = "SafeMode"
p_values = {"yaw": 1.0, "pitch": 1.0, "roll": 1.0}

FSM_STATES = ["SafeMode", "PanicMode", "ManualMode", "CalibrationMode", "YawControlMode",
              "FullControlMode", "RawMode"]

FSM_COLORS = {
    "SafeMode":        [100, 100, 255],
    "PanicMode":       [255, 60,  60 ],
    "ManualMode":      [100, 220, 100],
    "CalibrationMode": [220, 180, 50 ],
    "YawControlMode":  [80,  200, 200],
    "FullControlMode": [180, 100, 255],
    "RawMode":         [200, 200, 200],
}

start_time = time.time()


sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
sock.connect(SOCKET_PATH)
sock_file = sock.makefile("r")


# Serial reading thread
# ---------------------------------------

def serial_reader():
    global battery_level, fsm_state, joystick, p_values

    mock_fsm_index = 0
    mock_fsm_timer = time.time()
    mock_p_timer   = time.time()

    while True:
        try:
            if not MOCK_MODE:
                line = sock_file.readline()
                if not line:
                    continue

                t = json.loads(line)

                # print("JSON received:", t)

                if "state" in t and "bat_level" in t:
                    fsm_state = t["state"]
                    battery_level = t["bat_level"] / 100.0  # convert 0–100 to 0.0–1.0
                    continue

                yaw_data.append(t["yaw_rate"])
                pitch_data.append(t["pitch_rate"])
                roll_data.append(t["roll_rate"])
                time_data.append(time.time() - start_time)

                for i in range(4):
                    motor_values[i] = t["motors"][i]

                joystick["pitch"] = t["pitch"]
                joystick["roll"]  = t["roll"]
                joystick["lift"]  = t["lift"]
                joystick["yaw"]   = t["yaw"]

                battery_level = t["battery"]
                fsm_state     = t["fsm_state"]

                p_values["yaw"]   = t["p_yaw"]
                p_values["pitch"] = t["p_pitch"]
                p_values["roll"]  = t["p_roll"]
                # if ser.in_waiting:
                #     line = ser.readline().decode("utf-8", errors="ignore").strip()
                #     # Expected format:
                #     # yaw_rate,pitch_rate,roll_rate,m0,m1,m2,m3,
                #     # pitch,roll,lift,yaw,battery,fsm_state,
                #     # p_yaw,p_pitch,p_roll
                #     parts = line.split(",")
                #     if len(parts) == 16:
                #         t = time.time() - start_time
                #         yaw_data.append(float(parts[0]))
                #         pitch_data.append(float(parts[1]))
                #         roll_data.append(float(parts[2]))
                #         time_data.append(t)

                #         for i in range(4):
                #             motor_values[i] = int(parts[3 + i])

                #         joystick["pitch"] = float(parts[7])
                #         joystick["roll"]  = float(parts[8])
                #         joystick["lift"]  = float(parts[9])
                #         joystick["yaw"]   = float(parts[10])

                #         battery_level     = float(parts[11])
                #         fsm_state         = parts[12].strip()

                #         p_values["yaw"]   = float(parts[13])
                #         p_values["pitch"] = float(parts[14])
                #         p_values["roll"]  = float(parts[15])
            else:
                t = time.time() - start_time

                # Mock rates
                yaw_data.append(  math.sin(t * 2.0) * 50  + random.uniform(-5, 5))
                pitch_data.append(math.sin(t * 1.5) * 30  + random.uniform(-3, 3))
                roll_data.append( math.cos(t * 1.8) * 25  + random.uniform(-3, 3))
                time_data.append(t)

                # Mock motors
                for i in range(4):
                    motor_values[i] = int(400 + math.sin(t + i) * 200 + random.uniform(-20, 20))

                # Mock joystick
                joystick["pitch"] = round(math.sin(t * 0.7) * 0.8, 3)
                joystick["roll"]  = round(math.cos(t * 0.5) * 0.6, 3)
                joystick["lift"]  = round((math.sin(t * 0.3) + 1) / 2, 3)
                joystick["yaw"]   = round(math.sin(t * 1.1) * 0.9, 3)

                # Mock battery slowly draining
                battery_level = max(0.0, 1.0 - (t / 300.0))

                # Mock FSM cycling every 5 seconds
                if time.time() - mock_fsm_timer > 5:
                    mock_fsm_index = (mock_fsm_index + 1) % len(FSM_STATES)
                    fsm_state      = FSM_STATES[mock_fsm_index]
                    mock_fsm_timer = time.time()

                # Mock p_values slowly drifting
                if time.time() - mock_p_timer > 2:
                    p_values["yaw"]   = round(p_values["yaw"]   + random.uniform(-0.05, 0.05), 3)
                    p_values["pitch"] = round(p_values["pitch"] + random.uniform(-0.05, 0.05), 3)
                    p_values["roll"]  = round(p_values["roll"]  + random.uniform(-0.05, 0.05), 3)
                    mock_p_timer = time.time()

                time.sleep(0.05)

        except Exception as e:
            print(f"Serial error: {e}")


# GUI setup
# ---------------------------------------

dpg.create_context()

with dpg.window(label="Drone Ground Station", tag="main_window", width=900, height=800):

    if MOCK_MODE:
        dpg.add_text("⚠ MOCK MODE - No drone connected", tag="mock_warning", color=[255, 200, 0])

    dpg.add_separator()

    # --- FSM State ---
    dpg.add_text("FSM State")
    with dpg.group(horizontal=True):
        dpg.add_text("Current State:", color=[180, 180, 180])
        dpg.add_text("SafeMode", tag="fsm_display", color=FSM_COLORS["SafeMode"])

    dpg.add_separator()

    # --- Battery ---
    dpg.add_text("Battery Level")
    dpg.add_progress_bar(tag="battery_bar", default_value=1.0, width=400)
    dpg.add_text("100%", tag="battery_text")

    dpg.add_separator()

    # --- P values ---
    dpg.add_text("Controller P-Values")
    with dpg.table(header_row=True, borders_innerH=True, borders_outerH=True,
                   borders_innerV=True, borders_outerV=True):
        dpg.add_table_column(label="Controller")
        dpg.add_table_column(label="P Value")

        for axis in ["yaw", "pitch", "roll"]:
            with dpg.table_row():
                dpg.add_text(f"{axis.capitalize()} P:")
                dpg.add_text("1.000", tag=f"p_{axis}_display")

    dpg.add_separator()

    # --- Joystick Inputs ---
    dpg.add_text("Joystick Inputs")
    with dpg.table(header_row=True, borders_innerH=True, borders_outerH=True,
                   borders_innerV=True, borders_outerV=True):
        dpg.add_table_column(label="Axis")
        dpg.add_table_column(label="Value")
        dpg.add_table_column(label="Bar")

        for axis in ["pitch", "roll", "lift", "yaw"]:
            with dpg.table_row():
                dpg.add_text(axis.capitalize())
                dpg.add_text("0.000", tag=f"{axis}_val")
                dpg.add_progress_bar(tag=f"{axis}_bar", default_value=0.5, width=200)

    dpg.add_separator()

    # --- Motor outputs ---
    dpg.add_text("Motor Outputs")
    for i in range(4):
        with dpg.group(horizontal=True):
            dpg.add_text(f"M{i}:")
            dpg.add_progress_bar(tag=f"motor{i}", default_value=0, width=400)
            dpg.add_text("0", tag=f"motor{i}_val")

    dpg.add_separator()

    # --- Rate plots ---
    dpg.add_text("Sensor Rates")
    with dpg.plot(label="Yaw Rate", height=200, width=-1):
        dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis_yaw")
        dpg.add_plot_axis(dpg.mvYAxis, label="deg/s", tag="y_axis_yaw")
        dpg.add_line_series([], [], parent="y_axis_yaw", tag="yaw_series",
                            label="Yaw Rate")

    with dpg.plot(label="Pitch Rate", height=200, width=-1):
        dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis_pitch")
        dpg.add_plot_axis(dpg.mvYAxis, label="deg/s", tag="y_axis_pitch")
        dpg.add_line_series([], [], parent="y_axis_pitch", tag="pitch_series",
                            label="Pitch Rate")

    with dpg.plot(label="Roll Rate", height=200, width=-1):
        dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis_roll")
        dpg.add_plot_axis(dpg.mvYAxis, label="deg/s", tag="y_axis_roll")
        dpg.add_line_series([], [], parent="y_axis_roll", tag="roll_series",
                            label="Roll Rate")


# Update loop
# ---------------------------------------

def update_gui():
    while True:
        if len(time_data) > 0:
            t_list = list(time_data)
            x_min, x_max = t_list[0], t_list[-1]

            # Rate plots
            dpg.set_value("yaw_series",   [t_list, list(yaw_data)])
            dpg.set_value("pitch_series", [t_list, list(pitch_data)])
            dpg.set_value("roll_series",  [t_list, list(roll_data)])

            for axis in ["yaw", "pitch", "roll"]:
                dpg.set_axis_limits(f"x_axis_{axis}", x_min, x_max)

        # Motors
        for i in range(4):
            val = motor_values[i]
            dpg.set_value(f"motor{i}",     val / 800)
            dpg.set_value(f"motor{i}_val", str(val))

        # Joystick
        for axis in ["pitch", "roll", "lift", "yaw"]:
            val = joystick[axis]
            dpg.set_value(f"{axis}_val", f"{val:.3f}")
            dpg.set_value(f"{axis}_bar", (val + 1) / 2)

        # Battery
        dpg.set_value("battery_bar",  battery_level)
        dpg.set_value("battery_text", f"{battery_level * 100:.1f}%")
        dpg.configure_item("battery_bar", overlay=f"{battery_level * 100:.1f}%")

        # FSM state
        dpg.set_value("fsm_display", fsm_state)
        dpg.configure_item("fsm_display", color=FSM_COLORS.get(fsm_state, [255, 255, 255]))

        # P values
        for axis in ["yaw", "pitch", "roll"]:
            dpg.set_value(f"p_{axis}_display", f"{p_values[axis]:.3f}")

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