import math
import dearpygui.dearpygui as dpg

import util.data as stored_data
from util.states import COM_MODES, COM_MODES_NAMES, FSM_COLORS
from update_ui.update_time_diagrams import update_loop_timing
from update_ui.update_sensors import update_sensor_plots

YAW_RATE = 80

# Update loop
# ---------------------------------------

last_rendered_messages = None


def manual_value_to_bar(val: float, axis: str):
    if axis == "lift":
        return max(0.0, min(1.0, val / 1000.0))
    return max(0.0, min(1.0, (val + 1000.0) / 2000.0))


def update_battery_and_fsm():
    # FSM
    dpg.set_value("fsm_display", stored_data.fsm_state)
    dpg.configure_item(
        "fsm_display", color=FSM_COLORS.get(stored_data.fsm_state, [255, 255, 255])
    )

    dpg.set_value("packet_size_display", stored_data.telemetry_data_size)


def update_message_log():
    global last_rendered_messages

    if stored_data.pause_logs:
        return
    # Message log: only rebuild when the visible messages actually change,
    # otherwise the table keeps fighting the user's scroll position.
    with stored_data.message_log_lock:
        current_messages = tuple(stored_data.message_log)

    if current_messages != last_rendered_messages:
        dpg.delete_item("msg_table", children_only=True, slot=1)
        for ts, direction, msg in current_messages:
            color = [100, 220, 255] if direction == "Drone>PC" else [220, 180, 100]
            with dpg.table_row(parent="msg_table"):
                dpg.add_text(ts)
                dpg.add_text(direction, color=color)
                dpg.add_text(msg)
        last_rendered_messages = current_messages


def update_joystick():
    if len(stored_data.joystick["roll"]) == 0:
        return

    cx = 100 + (stored_data.joystick["roll"][-1]) * 90
    cy = 100 - (stored_data.joystick["pitch"][-1]) * 90
    dpg.configure_item("joystick_dot", center=[cx, cy])

    # lift bar — lift normalized 0-1
    lift_normalized = stored_data.joystick["lift"][-1]  # 0.0 to 1.0
    top = 190 - lift_normalized * 180  # pixels from top
    dpg.configure_item("lift_bar", pmin=[10, top], pmax=[30, 190])

    # yaw needle
    rad = math.radians(stored_data.joystick["yaw"][-1] * YAW_RATE)
    tip_x = 100 + 70 * math.sin(rad)
    tip_y = 100 - 70 * math.cos(rad)
    dpg.configure_item("yaw_needle", p1=[tip_x, tip_y], p2=[100, 100])


def update_pid_values():
    if len(stored_data.joystick["roll"]) == 0:
        return

    dpg.set_value("yaw_p_trim", f"{stored_data.joystick['yaw_p_trim'][-1]:.5f}")
    dpg.set_value(
        "roll_pitch_p_trim", f"{stored_data.joystick['roll_pitch_p_trim'][-1]:.5f}"
    )
    dpg.set_value(
        "roll_pitch_d_trim", f"{stored_data.joystick['roll_pitch_d_trim'][-1]:.5f}"
    )


def update_ble():
    dpg.set_value("ble_rssi", f"{stored_data.bluetooth['rssi']:.3f}dBm")

    dpg.set_value("com_mode", f"{COM_MODES_NAMES[stored_data.bluetooth['com_mode']]}")
    dpg.configure_item(
        "com_mode",
        color=COM_MODES.get(stored_data.bluetooth["com_mode"], [255, 255, 255]),
    )

    if stored_data.received_packages[0] != 0:
        dpg.set_value(
            "delta_packages",
            f"{(1000.0 * ((stored_data.received_packages[-1] - stored_data.received_packages[0]) / len(stored_data.received_packages))):.3f}ms",
        )


def update_step():
    update_joystick()
    update_pid_values()

    update_sensor_plots(stored_data.live_data, "_live")
    update_sensor_plots(stored_data.logged_data, "_logged")

    update_loop_timing(stored_data.live_data, "_live")
    update_loop_timing(stored_data.logged_data, "_logged")

    update_battery_and_fsm()
    update_message_log()

    update_ble()

    dpg.set_frame_callback(dpg.get_frame_count() + 3, update_step)
