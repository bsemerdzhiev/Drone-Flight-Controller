import sys
import serial
import threading
import time
import dearpygui.dearpygui as dpg
from collections import deque


# Serial connection
# ---------------------------------------

port = sys.argv[1]
ser = serial.Serial(port, 115200, timeout=0.1)


# Telemetry storage
# ---------------------------------------

yaw_data = deque(maxlen=200)
time_data = deque(maxlen=200)

motor_values = [0, 0, 0, 0]

start_time = time.time()


# Serial reading thread
# ---------------------------------------

def serial_reader():
    while True:
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="ignore").strip()

                
                # yaw_rate,m0,m1,m2,m3
                parts = line.split(",")

                if len(parts) == 5:
                    yaw_rate = float(parts[0])
                    motors = [int(x) for x in parts[1:]]

                    t = time.time() - start_time

                    yaw_data.append(yaw_rate)
                    time_data.append(t)

                    for i in range(4):
                        motor_values[i] = motors[i]

        except:
            pass



# GUI setup
# ---------------------------------------

dpg.create_context()

with dpg.window(label="Drone Ground Station", width=900, height=700):

    dpg.add_text("Drone Status")

    dpg.add_separator()

    # Controller parameters
    with dpg.group(horizontal=True):
        dpg.add_text("Yaw P:")
        dpg.add_text("1.0", tag="kp_display")

    dpg.add_separator()

    # Motor outputs
    dpg.add_text("Motor Outputs")

    dpg.add_progress_bar(tag="motor0", default_value=0)
    dpg.add_progress_bar(tag="motor1", default_value=0)
    dpg.add_progress_bar(tag="motor2", default_value=0)
    dpg.add_progress_bar(tag="motor3", default_value=0)

    dpg.add_separator()

    # Plot
    dpg.add_text("Yaw Rate")

    with dpg.plot(label="Yaw Rate Plot", height=300, width=-1):

        dpg.add_plot_axis(dpg.mvXAxis, label="time", tag="x_axis")
        y_axis = dpg.add_plot_axis(dpg.mvYAxis, label="rate", tag="y_axis")

        dpg.add_line_series([], [], parent="y_axis", tag="yaw_series")


# Update loop
# ---------------------------------------

def update_gui():
    while True:

        if len(time_data) > 0:
            dpg.set_value("yaw_series", [list(time_data), list(yaw_data)])

        # Update motor bars
        for i in range(4):
            dpg.set_value(f"motor{i}", motor_values[i] / 800)

        time.sleep(0.05)



# Launch threads
# ---------------------------------------

threading.Thread(target=serial_reader, daemon=True).start()
threading.Thread(target=update_gui, daemon=True).start()


# Start GUI
# ---------------------------------------

dpg.create_viewport(title="Drone Interface", width=900, height=700)
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()