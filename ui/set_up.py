import dearpygui.dearpygui as dpg

from states import FSM_COLORS
import data as stored_data


def toggle_pause(label_suffix: str):
    if label_suffix == "_live":
        stored_data.live_data.is_paused ^= True
    else:
        stored_data.logged_data.is_paused ^= True


def toggle_pause_logs():
    stored_data.pause_logs ^= True


def set_up_sensors(label_suffix: str):
    dpg.add_button(
        label="Pause",
        tag="pause_btn" + label_suffix,
        callback=lambda: toggle_pause(label_suffix),
    )
    dpg.add_text("Sensor Feed", color=[255, 255, 100])
    for sensor_label, axes in [
        (
            "Accelerometer",
            [
                (
                    "Accel X",
                    "x_axis_accel_x",
                    "y_axis_accel_x",
                    "accel_x_series",
                    "i16",
                ),
                (
                    "Accel Y",
                    "x_axis_accel_y",
                    "y_axis_accel_y",
                    "accel_y_series",
                    "i16",
                ),
                (
                    "Accel Z",
                    "x_axis_accel_z",
                    "y_axis_accel_z",
                    "accel_z_series",
                    "i16",
                ),
            ],
        ),
        (
            "Gyroscope",
            [
                ("Gyro X", "x_axis_gyro_x", "y_axis_gyro_x", "gyro_x_series", "i16"),
                ("Gyro Y", "x_axis_gyro_y", "y_axis_gyro_y", "gyro_y_series", "i16"),
                ("Gyro Z", "x_axis_gyro_z", "y_axis_gyro_z", "gyro_z_series", "i16"),
            ],
        ),
        (
            "Barometer",
            [
                ("Pressure", "x_axis_baro", "y_axis_baro", "baro_series", "hPa"),
            ],
        ),
        (
            "Battery readings",
            [
                (
                    "Battery",
                    "x_axis_battery",
                    "y_axis_battery",
                    "battery_series",
                    "Voltage * 10^-2",
                ),
            ],
        ),
    ]:
        dpg.add_text(sensor_label, color=[180, 180, 180])
        with dpg.group(horizontal=True):
            for label, tag_x, tag_y, series_tag, y_label in axes:
                with dpg.plot(label=label, height=180, width=380):
                    dpg.add_plot_axis(
                        dpg.mvXAxis, label="time", tag=tag_x + label_suffix
                    )
                    dpg.add_plot_axis(
                        dpg.mvYAxis, label=y_label, tag=tag_y + label_suffix
                    )
                    dpg.add_line_series(
                        [],
                        [],
                        parent=tag_y + label_suffix,
                        tag=series_tag + label_suffix,
                    )
        dpg.add_separator()

    # Rates — two series per plot
    dpg.add_text("Rates", color=[180, 180, 180])
    with dpg.group(horizontal=True):
        for label, tag_x, tag_y, dmp_tag, kalman_tag in [
            ("Yaw", "x_axis_yaw", "y_axis_yaw", "yaw_series_dmp", "yaw_series_kalman"),
            (
                "Pitch",
                "x_axis_pitch",
                "y_axis_pitch",
                "pitch_series_dmp",
                "pitch_series_kalman",
            ),
            (
                "Roll",
                "x_axis_roll",
                "y_axis_roll",
                "roll_series_dmp",
                "roll_series_kalman",
            ),
        ]:
            with dpg.plot(label=label, height=180, width=380):
                dpg.add_plot_axis(dpg.mvXAxis, label="time", tag=tag_x + label_suffix)
                dpg.add_plot_axis(dpg.mvYAxis, label="deg", tag=tag_y + label_suffix)
                dpg.add_line_series(
                    [],
                    [],
                    label="DMP",
                    parent=tag_y + label_suffix,
                    tag=dmp_tag + label_suffix,
                )
                dpg.add_line_series(
                    [],
                    [],
                    label="Kalman",
                    parent=tag_y + label_suffix,
                    tag=kalman_tag + label_suffix,
                )
    # dpg.add_plot_legend(
    #     parent="y_axis_yaw" + label_suffix
    # )  # optional, adds legend to yaw
    dpg.add_separator()

    dpg.set_axis_limits("y_axis_yaw" + label_suffix, -150, 150)
    dpg.configure_item("y_axis_yaw" + label_suffix, no_initial_fit=True)
    dpg.set_axis_limits("y_axis_pitch" + label_suffix, -150, 150)
    dpg.configure_item("y_axis_pitch" + label_suffix, no_initial_fit=True)
    dpg.set_axis_limits("y_axis_roll" + label_suffix, -150, 150)
    dpg.configure_item("y_axis_roll" + label_suffix, no_initial_fit=True)


def set_up_gui():
    dpg.create_context()

    with dpg.window(label="Drone Live Feed", tag="main_window", width=900, height=800):
        # --- FSM State ---
        dpg.add_text("FSM State", color=[255, 255, 100])
        with dpg.group(horizontal=True):
            dpg.add_text("Current State:", color=[180, 180, 180])
            dpg.add_text("SafeMode", tag="fsm_display", color=FSM_COLORS["SafeMode"])
        dpg.add_separator()
        # --- Battery ---
        dpg.add_text("Battery Level", color=[255, 255, 100])
        dpg.add_progress_bar(
            tag="battery_bar", default_value=1.0, width=400, overlay="100%"
        )
        dpg.add_text("100%", tag="battery_text")
        dpg.add_separator()

        # --- Joystick ---
        dpg.add_text("Joystick", color=[255, 255, 100])

        with dpg.group(horizontal=True):
            # Pitch / Roll 2D pad
            with dpg.group():
                dpg.add_text("Pitch / Roll", color=[180, 180, 180])
                with dpg.drawlist(width=200, height=200, tag="joystick_draw"):
                    dpg.draw_circle(
                        [100, 100], 90, color=[80, 80, 80], fill=[30, 30, 30]
                    )
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
                    dpg.draw_circle(
                        [100, 100], 90, color=[80, 80, 80], fill=[30, 30, 30]
                    )
                    # cardinal marks
                    for angle, label, pos in [
                        (0, "N", [97, 8]),
                        (90, "E", [183, 97]),
                        (180, "S", [97, 183]),
                        (270, "W", [5, 97]),
                    ]:
                        dpg.draw_text(pos, label, color=[150, 150, 150], size=13)
                    # tick marks
                    import math

                    for deg in range(0, 360, 30):
                        rad = math.radians(deg)
                        x1 = 100 + 80 * math.sin(rad)
                        y1 = 100 - 80 * math.cos(rad)
                        x2 = 100 + 90 * math.sin(rad)
                        y2 = 100 - 90 * math.cos(rad)
                        dpg.draw_line([x1, y1], [x2, y2], color=[80, 80, 80])
                    # needle
                    dpg.draw_arrow(
                        [100, 100 - 70],
                        [100, 100],
                        color=[0, 200, 255],
                        thickness=2,
                        tag="yaw_needle",
                    )

            # P/D trim columns
            with dpg.group():
                dpg.add_text("Trim Values", color=[180, 180, 180])
                with dpg.table(
                    header_row=True,
                    borders_innerV=True,
                    borders_outerV=True,
                    borders_innerH=True,
                    borders_outerH=True,
                ):
                    dpg.add_table_column(label="Yaw P")
                    dpg.add_table_column(label="Roll/Pitch P")
                    dpg.add_table_column(label="Roll/Pitch D")
                    with dpg.table_row():
                        dpg.add_text("0.000", tag="yaw_p_trim")
                        dpg.add_text("0.000", tag="roll_pitch_p_trim")
                        dpg.add_text("0.000", tag="roll_pitch_d_trim")

        dpg.add_separator()
        set_up_sensors("_live")
    with dpg.window(
        label="Drone Logged Feed", tag="logged_feed", width=900, height=800
    ):
        set_up_sensors("_logged")

    with dpg.window(label="All Messages Log", tag="logs", width=900, height=800):
        dpg.add_button(
            label="Pause",
            tag="pause_btn_logs",
            callback=lambda: toggle_pause_logs(),
        )

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
            height=560,
        ):
            dpg.add_table_column(
                label="Time", width_fixed=True, init_width_or_weight=70
            )
            dpg.add_table_column(
                label="Direction", width_fixed=True, init_width_or_weight=90
            )
            dpg.add_table_column(label="Message")
