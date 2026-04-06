import dearpygui.dearpygui as dpg

from util.states import FSM_COLORS, SENSOR_NAMES
import util.data as stored_data


def set_up_sensors(label_suffix: str):
    dpg.add_text("Sensor Feed", color=[255, 255, 100])

    # --- Single-series sensor plots ---
    for sensor_label, axes in [
        (
            "Accelerometer",
            [
                (
                    "Accel X",
                    "x_axis_accel_x",
                    "y_axis_accel_x",
                    "accel_x_series",
                    "G(m/s^2)",
                ),
                (
                    "Accel Y",
                    "x_axis_accel_y",
                    "y_axis_accel_y",
                    "accel_y_series",
                    "G(m/s^2)",
                ),
                (
                    "Accel Z",
                    "x_axis_accel_z",
                    "y_axis_accel_z",
                    "accel_z_series",
                    "G(m/s^2)",
                ),
            ],
        ),
        (
            "Gyroscope",
            [
                ("Gyro X", "x_axis_gyro_x", "y_axis_gyro_x", "gyro_x_series", "deg/s"),
                ("Gyro Y", "x_axis_gyro_y", "y_axis_gyro_y", "gyro_y_series", "deg/s"),
                ("Gyro Z", "x_axis_gyro_z", "y_axis_gyro_z", "gyro_z_series", "deg/s"),
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

    # --- Motors ---
    dpg.add_text("Motors", color=[180, 180, 180])
    with dpg.group(horizontal=True):
        for i in range(4):
            with dpg.plot(label=f"Motor {i}", height=180, width=380):
                dpg.add_plot_axis(
                    dpg.mvXAxis, label="time", tag=f"x_axis_motor_{i}" + label_suffix
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="pwm", tag=f"y_axis_motor_{i}" + label_suffix
                )
                dpg.add_line_series(
                    [],
                    [],
                    label=f"Motor {i}",
                    parent=f"y_axis_motor_{i}" + label_suffix,
                    tag=f"motor_{i}_series" + label_suffix,
                )
    dpg.add_separator()

    # --- Barometer (two series: raw + Kalman) ---
    dpg.add_text("Barometer", color=[180, 180, 180])
    with dpg.group(horizontal=True):
        with dpg.plot(label="Pressure Together", height=180, width=380):
            dpg.add_plot_axis(
                dpg.mvXAxis, label="time", tag="x_axis_baro" + label_suffix
            )
            dpg.add_plot_axis(dpg.mvYAxis, label="m", tag="y_axis_baro" + label_suffix)
            dpg.add_line_series(
                [],
                [],
                label="Raw",
                parent="y_axis_baro" + label_suffix,
                tag="baro_series_raw" + label_suffix,
            )
            dpg.add_line_series(
                [],
                [],
                label="Kalman",
                parent="y_axis_baro" + label_suffix,
                tag="baro_series_kalman" + label_suffix,
            )
        with dpg.plot(label="Pressure Raw", height=180, width=380):
            dpg.add_plot_axis(
                dpg.mvXAxis, label="time", tag="x_axis_baro_raw" + label_suffix
            )
            dpg.add_plot_axis(
                dpg.mvYAxis, label="m", tag="y_axis_baro_raw" + label_suffix
            )
            dpg.add_line_series(
                [],
                [],
                label="Raw",
                parent="y_axis_baro_raw" + label_suffix,
                tag="baro_series_solo_raw" + label_suffix,
            )
        with dpg.plot(label="Pressure Filtered", height=180, width=380):
            dpg.add_plot_axis(
                dpg.mvXAxis, label="time", tag="x_axis_baro_kalman" + label_suffix
            )
            dpg.add_plot_axis(
                dpg.mvYAxis, label="m", tag="y_axis_baro_kalman" + label_suffix
            )
            dpg.add_line_series(
                [],
                [],
                label="Kalman",
                parent="y_axis_baro_kalman" + label_suffix,
                tag="baro_series_solo_kalman" + label_suffix,
            )
            dpg.add_line_series(
                [],
                [],
                label="Target",
                parent="y_axis_baro_kalman" + label_suffix,
                tag="baro_target_line" + label_suffix,
            )

    dpg.add_separator()

    # --- Rates (three plots each: combined, DMP-only, Kalman-only) ---
    dpg.add_text("Rates", color=[180, 180, 180])
    for (
        rate_label,
        tag_x,
        tag_y,
        dmp_tag,
        kalman_tag,
        tag_x_dmp,
        tag_y_dmp,
        tag_x_kal,
        tag_y_kal,
        solo_dmp_tag,
        solo_kal_tag,
    ) in [
        (
            "Yaw",
            "x_axis_yaw",
            "y_axis_yaw",
            "yaw_series_dmp",
            "yaw_series_kalman",
            "x_axis_yaw_dmp",
            "y_axis_yaw_dmp",
            "x_axis_yaw_kal",
            "y_axis_yaw_kal",
            "yaw_series_solo_dmp",
            "yaw_series_solo_kal",
        ),
        (
            "Pitch",
            "x_axis_pitch",
            "y_axis_pitch",
            "pitch_series_dmp",
            "pitch_series_kalman",
            "x_axis_pitch_dmp",
            "y_axis_pitch_dmp",
            "x_axis_pitch_kal",
            "y_axis_pitch_kal",
            "pitch_series_solo_dmp",
            "pitch_series_solo_kal",
        ),
        (
            "Roll",
            "x_axis_roll",
            "y_axis_roll",
            "roll_series_dmp",
            "roll_series_kalman",
            "x_axis_roll_dmp",
            "y_axis_roll_dmp",
            "x_axis_roll_kal",
            "y_axis_roll_kal",
            "roll_series_solo_dmp",
            "roll_series_solo_kal",
        ),
    ]:
        dpg.add_text(rate_label, color=[150, 150, 150])
        # Row 1: combined plot + DMP-only + Kalman-only
        with dpg.group(horizontal=True):
            # Combined (DMP + Kalman overlaid)
            with dpg.plot(label=rate_label + " Together", height=180, width=380):
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
            # DMP only
            with dpg.plot(label=rate_label + " DMP", height=180, width=380):
                dpg.add_plot_axis(
                    dpg.mvXAxis, label="time", tag=tag_x_dmp + label_suffix
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="deg", tag=tag_y_dmp + label_suffix
                )
                dpg.add_line_series(
                    [],
                    [],
                    label="DMP",
                    parent=tag_y_dmp + label_suffix,
                    tag=solo_dmp_tag + label_suffix,
                )
            # Kalman only
            with dpg.plot(label=rate_label + " Filtered", height=180, width=380):
                dpg.add_plot_axis(
                    dpg.mvXAxis, label="time", tag=tag_x_kal + label_suffix
                )
                dpg.add_plot_axis(
                    dpg.mvYAxis, label="deg", tag=tag_y_kal + label_suffix
                )
                dpg.add_line_series(
                    [],
                    [],
                    label="Kalman",
                    parent=tag_y_kal + label_suffix,
                    tag=solo_kal_tag + label_suffix,
                )

    dpg.add_separator()

    # Lock Y axis limits for all rate plots
    for axis in ["yaw", "pitch", "roll"]:
        for suffix in [
            "",
        ]:
            tag = f"y_axis_{axis}{suffix}" + label_suffix
            dpg.set_axis_limits(tag, -150, 150)
            dpg.configure_item(tag, no_initial_fit=True)
