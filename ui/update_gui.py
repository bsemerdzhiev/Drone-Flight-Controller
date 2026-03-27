import time
import dearpygui.dearpygui as dpg

import data as stored_data
from states import FSM_COLORS

# Update loop
# ---------------------------------------


def manual_value_to_bar(val: float, axis: str):
    if axis == "lift":
        return max(0.0, min(1.0, val / 1000.0))
    return max(0.0, min(1.0, (val + 1000.0) / 2000.0))


def update_gui():
    last_rendered_messages = None

    while True:
        if len(stored_data.time_data) > 0:
            t_list = list(stored_data.time_data)
            x_min, x_max = t_list[0], t_list[-1]
            rate_series = {
                "yaw": list(stored_data.yaw_data),
                "pitch": list(stored_data.pitch_data),
                "roll": list(stored_data.roll_data),
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

        # Battery
        dpg.set_value("battery_bar", stored_data.battery_level)
        dpg.set_value("battery_text", f"{stored_data.battery_level * 100:.1f}%")
        dpg.configure_item(
            "battery_bar", overlay=f"{stored_data.battery_level * 100:.1f}%"
        )

        # FSM
        dpg.set_value("fsm_display", stored_data.fsm_state)
        dpg.configure_item(
            "fsm_display", color=FSM_COLORS.get(stored_data.fsm_state, [255, 255, 255])
        )

        # Accel & Gyro
        for sensor, data in [
            ("accel", stored_data.accel_raw),
            ("gyro", stored_data.gyro_raw),
        ]:
            for axis in ["x", "y", "z"]:
                dpg.set_value(f"{sensor}_{axis}", str(data[axis]))

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

        time.sleep(0.05)
