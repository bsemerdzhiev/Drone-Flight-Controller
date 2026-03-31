import math
import time
import dearpygui.dearpygui as dpg

import data as stored_data
from states import FSM_COLORS

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


def fit_with_margin(y_axis_tag: str, data: list, margin: float = 0.2):
    if not data:
        return
    lo, hi = min(data), max(data)
    span = hi - lo
    if span == 0:
        span = abs(lo) if lo != 0 else 1.0
    pad = span * margin

    dpg.set_axis_limits(y_axis_tag, lo - pad, hi + pad)


def update_sensor_plots(read_data: stored_data.ReadData, label_suffix: str):
    t = list(read_data.time_data)

    if len(t) == 0 or read_data.is_paused:
        return

    # Position
    # DMP
    dpg.set_value("yaw_series_dmp" + label_suffix, [t, list(read_data.yaw_data)])
    dpg.set_value("yaw_series_solo_dmp" + label_suffix, [t, list(read_data.yaw_data)])

    dpg.set_value("pitch_series_dmp" + label_suffix, [t, list(read_data.pitch_data)])
    dpg.set_value(
        "pitch_series_solo_dmp" + label_suffix, [t, list(read_data.pitch_data)]
    )

    dpg.set_value("roll_series_dmp" + label_suffix, [t, list(read_data.roll_data)])
    dpg.set_value("roll_series_solo_dmp" + label_suffix, [t, list(read_data.roll_data)])

    # Kalman
    dpg.set_value("yaw_series_kalman" + label_suffix, [t, list(read_data.yaw_kalman)])
    dpg.set_value("yaw_series_solo_kal" + label_suffix, [t, list(read_data.yaw_kalman)])

    dpg.set_value(
        "pitch_series_kalman" + label_suffix, [t, list(read_data.pitch_kalman)]
    )
    dpg.set_value(
        "pitch_series_solo_kal" + label_suffix, [t, list(read_data.pitch_kalman)]
    )

    dpg.set_value("roll_series_kalman" + label_suffix, [t, list(read_data.roll_kalman)])
    dpg.set_value(
        "roll_series_solo_kal" + label_suffix, [t, list(read_data.roll_kalman)]
    )

    dpg.fit_axis_data("x_axis_yaw" + label_suffix)
    dpg.fit_axis_data("x_axis_pitch" + label_suffix)
    dpg.fit_axis_data("x_axis_roll" + label_suffix)

    dpg.fit_axis_data("x_axis_yaw_dmp" + label_suffix)
    dpg.fit_axis_data("x_axis_pitch_dmp" + label_suffix)
    dpg.fit_axis_data("x_axis_roll_dmp" + label_suffix)

    dpg.fit_axis_data("x_axis_yaw_kal" + label_suffix)
    dpg.fit_axis_data("x_axis_pitch_kal" + label_suffix)
    dpg.fit_axis_data("x_axis_roll_kal" + label_suffix)

    fit_with_margin("y_axis_yaw_kal" + label_suffix, list(read_data.yaw_kalman))
    fit_with_margin("y_axis_pitch_kal" + label_suffix, list(read_data.pitch_kalman))
    fit_with_margin("y_axis_roll_kal" + label_suffix, list(read_data.roll_kalman))

    fit_with_margin("y_axis_yaw_dmp" + label_suffix, list(read_data.yaw_data))
    fit_with_margin("y_axis_pitch_dmp" + label_suffix, list(read_data.pitch_data))
    fit_with_margin("y_axis_roll_dmp" + label_suffix, list(read_data.roll_data))

    # fit_with_margin("y_axis_yaw_kal" + label_suffix, list(read_data.yaw_kalman[-1]))
    # fit_with_margin("y_axis_pitch_kal" + label_suffix, list(read_data.pitch_kalman[-1]))
    # fit_with_margin("y_axis_roll_kal" + label_suffix, list(read_data.roll_kalman[-1]))

    # Accel
    dpg.set_value("accel_x_series" + label_suffix, [t, list(read_data.accel_raw["x"])])
    dpg.set_value("accel_y_series" + label_suffix, [t, list(read_data.accel_raw["y"])])
    dpg.set_value("accel_z_series" + label_suffix, [t, list(read_data.accel_raw["z"])])
    dpg.fit_axis_data("x_axis_accel_x" + label_suffix)
    dpg.fit_axis_data("x_axis_accel_y" + label_suffix)
    dpg.fit_axis_data("x_axis_accel_z" + label_suffix)

    fit_with_margin("y_axis_accel_x" + label_suffix, list(read_data.accel_raw["x"]))
    fit_with_margin("y_axis_accel_y" + label_suffix, list(read_data.accel_raw["y"]))
    fit_with_margin("y_axis_accel_z" + label_suffix, list(read_data.accel_raw["z"]))

    # Gyro
    dpg.set_value("gyro_x_series" + label_suffix, [t, list(read_data.gyro_raw["x"])])
    dpg.set_value("gyro_y_series" + label_suffix, [t, list(read_data.gyro_raw["y"])])
    dpg.set_value("gyro_z_series" + label_suffix, [t, list(read_data.gyro_raw["z"])])
    dpg.fit_axis_data("x_axis_gyro_x" + label_suffix)
    dpg.fit_axis_data("x_axis_gyro_y" + label_suffix)
    dpg.fit_axis_data("x_axis_gyro_z" + label_suffix)

    fit_with_margin("y_axis_gyro_x" + label_suffix, list(read_data.gyro_raw["x"]))
    fit_with_margin("y_axis_gyro_y" + label_suffix, list(read_data.gyro_raw["y"]))
    fit_with_margin("y_axis_gyro_z" + label_suffix, list(read_data.gyro_raw["z"]))

    # Barometer
    dpg.set_value("baro_series_raw" + label_suffix, [t, list(read_data.pres_data)])
    dpg.set_value(
        "baro_series_kalman" + label_suffix, [t, list(read_data.pres_data_filtered)]
    )
    dpg.fit_axis_data("x_axis_baro" + label_suffix)

    fit_with_margin(
        "y_axis_baro" + label_suffix,
        list(read_data.pres_data) + list(read_data.pres_data_filtered),
    )
    # -------------------------------------------------

    dpg.set_value("baro_series_solo_raw" + label_suffix, [t, list(read_data.pres_data)])
    dpg.fit_axis_data("x_axis_baro_raw" + label_suffix)

    fit_with_margin(
        "y_axis_baro_raw" + label_suffix,
        list(read_data.pres_data),
    )

    # -------------------------------------------------

    for i in range(4):
        dpg.set_value(
            f"motor_{i}_series" + label_suffix, [t, list(read_data.motors[i])]
        )
        dpg.fit_axis_data(f"x_axis_motor_{i}" + label_suffix)

        fit_with_margin(f"y_axis_motor_{i}" + label_suffix, list(read_data.motors[i]))

    # -------------------------------------------------

    dpg.set_value(
        "baro_series_solo_kalman" + label_suffix,
        [t, list(read_data.pres_data_filtered)],
    )
    dpg.fit_axis_data("x_axis_baro_kalman" + label_suffix)

    fit_with_margin(
        "y_axis_baro_kalman" + label_suffix,
        list(read_data.pres_data_filtered),
    )

    # Battery
    dpg.set_value("battery_series" + label_suffix, [t, list(read_data.battery_level)])
    dpg.fit_axis_data("x_axis_battery" + label_suffix)

    fit_with_margin("y_axis_battery" + label_suffix, list(read_data.battery_level))


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

    dpg.set_value("yaw_p_trim", f"{stored_data.joystick['yaw_p_trim'][-1]:.3f}")
    dpg.set_value(
        "roll_pitch_p_trim", f"{stored_data.joystick['roll_pitch_p_trim'][-1]:.3f}"
    )
    dpg.set_value(
        "roll_pitch_d_trim", f"{stored_data.joystick['roll_pitch_d_trim'][-1]:.3f}"
    )


def update_step():
    update_joystick()
    update_pid_values()

    update_sensor_plots(stored_data.live_data, "_live")
    update_sensor_plots(stored_data.logged_data, "_logged")

    # if len(stored_data.time_data) > 0:
    #     t_list = list(stored_data.time_data)
    #     x_min, x_max = t_list[0], t_list[-1]
    #     rate_series = {
    #         "yaw": list(stored_data.yaw_data),
    #         "pitch": list(stored_data.pitch_data),
    #         "roll": list(stored_data.roll_data),
    #     }
    #
    #     dpg.set_value("yaw_series", [t_list, rate_series["yaw"]])
    #     dpg.set_value("pitch_series", [t_list, rate_series["pitch"]])
    #     dpg.set_value("roll_series", [t_list, rate_series["roll"]])
    #
    #     for axis in ["yaw", "pitch", "roll"]:
    #         dpg.set_axis_limits(f"x_axis_{axis}", x_min, x_max)
    #         max_abs_rate = max((abs(v) for v in rate_series[axis]), default=1.0)
    #         y_limit = max(1.0, max_abs_rate * 1.1)
    #         dpg.set_axis_limits(f"y_axis_{axis}", -y_limit, y_limit)
    #
    # Motors
    # for i in range(4):
    #     val = stored_data.motor_values[i]
    #     dpg.set_value(f"motor{i}", val / 800)
    #     dpg.set_value(f"motor{i}_val", str(val))
    #
    # Joystick
    # for axis in ["pitch", "roll", "lift", "yaw"]:
    #     val = stored_data.joystick[axis]
    #     dpg.set_value(f"{axis}_val", f"{val:.3f}")
    #     dpg.set_value(f"{axis}_bar", manual_value_to_bar(val, axis))
    #

    update_battery_and_fsm()
    #
    # # Accel & Gyro
    # for sensor, data in [
    #     ("accel", stored_data.accel_raw),
    #     ("gyro", stored_data.gyro_raw),
    # ]:
    #     for axis in ["x", "y", "z"]:
    #         dpg.set_value(f"{sensor}_{axis}", str(data[axis]))
    #

    update_message_log()

    dpg.set_frame_callback(dpg.get_frame_count() + 3, update_step)
