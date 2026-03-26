import time
import dearpygui.dearpygui as dpg

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
