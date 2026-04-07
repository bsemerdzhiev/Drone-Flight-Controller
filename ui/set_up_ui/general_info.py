import dearpygui.dearpygui as dpg
import numpy

from set_up_ui.set_up_pid import set_up_pid
from util.states import FSM_COLORS, SENSOR_NAMES
import util.data as stored_data
from set_up_ui.set_up_sensors import set_up_sensors


def choose_sensors():
    dpg.add_text("Visualization Sensors", color=[255, 255, 100])
    with dpg.group(horizontal=True):
        dpg.add_text("Current sensors", color=[180, 180, 180])
        dpg.add_text("DMP", tag="chosen_sensors_text", color=SENSOR_NAMES["DMP"])

    dpg.add_separator()


def show_fsm():
    # --- FSM State ---
    dpg.add_text("FSM State", color=[255, 255, 100])
    with dpg.group(horizontal=True):
        dpg.add_text("Current State:", color=[180, 180, 180])
        dpg.add_text("SafeMode", tag="fsm_display", color=FSM_COLORS["SafeMode"])
    dpg.add_separator()


def show_packet_size():
    dpg.add_text("Communication Packet Size", color=[255, 255, 100])
    with dpg.group(horizontal=True):
        dpg.add_text("Packet size:", color=[180, 180, 180])
        dpg.add_text("0", tag="packet_size_display", color=[180, 180, 180])

    dpg.add_separator()


def show_joystick():
    # --- Joystick ---
    dpg.add_text("Joystick", color=[255, 255, 100])

    with dpg.group(horizontal=True):
        # Pitch / Roll 2D pad
        with dpg.group():
            dpg.add_text("Pitch / Roll", color=[180, 180, 180])
            with dpg.drawlist(width=200, height=200, tag="joystick_draw"):
                dpg.draw_circle([100, 100], 90, color=[80, 80, 80], fill=[30, 30, 30])
                dpg.draw_line([10, 100], [190, 100], color=[60, 60, 60])
                dpg.draw_line([100, 10], [100, 190], color=[60, 60, 60])
                dpg.draw_circle(
                    [100, 100],
                    8,
                    color=[0, 200, 255],
                    fill=[0, 200, 255],
                    tag="joystick_dot",
                )

        # Lift bar (vertical)
        with dpg.group():
            dpg.add_text("Lift", color=[180, 180, 180])
            with dpg.drawlist(width=40, height=200, tag="lift_draw"):
                dpg.draw_rectangle(
                    [10, 10], [30, 190], color=[80, 80, 80], fill=[30, 30, 30]
                )
                dpg.draw_rectangle(
                    [10, 190],
                    [30, 190],
                    color=[0, 200, 255],
                    fill=[0, 200, 255],
                    tag="lift_bar",
                )

        # Yaw compass
        with dpg.group():
            dpg.add_text("Yaw", color=[180, 180, 180])
            with dpg.drawlist(width=200, height=200, tag="yaw_draw"):
                dpg.draw_circle([100, 100], 90, color=[80, 80, 80], fill=[30, 30, 30])
                for angle, label, pos in [
                    (0, "N", [97, 8]),
                    (90, "E", [183, 97]),
                    (180, "S", [97, 183]),
                    (270, "W", [5, 97]),
                ]:
                    dpg.draw_text(pos, label, color=[150, 150, 150], size=13)
                import math

                for deg in range(0, 360, 30):
                    rad = math.radians(deg)
                    x1 = 100 + 80 * math.sin(rad)
                    y1 = 100 - 80 * math.cos(rad)
                    x2 = 100 + 90 * math.sin(rad)
                    y2 = 100 - 90 * math.cos(rad)
                    dpg.draw_line([x1, y1], [x2, y2], color=[80, 80, 80])
                dpg.draw_arrow(
                    [100, 100 - 70],
                    [100, 100],
                    color=[0, 200, 255],
                    thickness=2,
                    tag="yaw_needle",
                )

    dpg.add_separator()


def show_ble_info():
    dpg.add_text("Bluetooth Info", color=[255, 255, 100])
    with dpg.group(horizontal=True):
        dpg.add_text("RSSI:", color=[180, 180, 180])
        dpg.add_text("Not Connected", tag="ble_rssi", color=[220, 180, 50])
    with dpg.group(horizontal=True):
        dpg.add_text("Communication Mode:", color=[180, 180, 180])
        dpg.add_text("UART", tag="com_mode", color=[180, 180, 180])

    with dpg.group(horizontal=True):
        dpg.add_text("Time Between Packages:", color=[180, 180, 180])
        dpg.add_text("0", tag="delta_packages", color=[180, 100, 255])

    dpg.add_separator()


def set_up_general_info():
    with dpg.tab(label="General Info"):
        with dpg.child_window(height=-1, border=False):
            choose_sensors()
            show_fsm()
            show_packet_size()
            show_joystick()
            show_ble_info()
            set_up_pid()
