import dearpygui.dearpygui as dpg

import util.data as stored_data


def fit_with_margin(y_axis_tag: str, data: list, margin: float = 0.2):
    if not data:
        return
    lo, hi = min(data), max(data)
    span = hi - lo
    if span == 0:
        span = abs(lo) if lo != 0 else 1.0
    pad = span * margin

    dpg.set_axis_limits(y_axis_tag, lo - pad, hi + pad)


def update_position(read_data: stored_data.ReadData, label_suffix: str, t: list):
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


def update_raw_readings(read_data: stored_data.ReadData, label_suffix: str, t: list):
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


def update_baro_readings(read_data: stored_data.ReadData, label_suffix: str, t: list):
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

    dpg.set_value(
        "baro_series_solo_kalman" + label_suffix,
        [t, list(read_data.pres_data_filtered)],
    )
    dpg.fit_axis_data("x_axis_baro_kalman" + label_suffix)

    dpg.set_value(
        "baro_target_line" + label_suffix,
        [t, list(read_data.pid_info["selected_height"])],
    )

    fit_with_margin(
        "y_axis_baro_kalman" + label_suffix,
        list(read_data.pres_data_filtered),
    )


def update_motor_readings(read_data: stored_data.ReadData, label_suffix: str, t: list):
    for i in range(4):
        dpg.set_value(
            f"motor_{i}_series" + label_suffix, [t, list(read_data.motors[i])]
        )
        dpg.fit_axis_data(f"x_axis_motor_{i}" + label_suffix)

        fit_with_margin(f"y_axis_motor_{i}" + label_suffix, list(read_data.motors[i]))


def update_battery_readings(
    read_data: stored_data.ReadData, label_suffix: str, t: list
):
    # Battery
    dpg.set_value("battery_series" + label_suffix, [t, list(read_data.battery_level)])
    dpg.fit_axis_data("x_axis_battery" + label_suffix)

    fit_with_margin("y_axis_battery" + label_suffix, list(read_data.battery_level))


def update_sensor_plots(read_data: stored_data.ReadData, label_suffix: str):
    t = list(read_data.time_data)

    if len(t) == 0 or stored_data.pause_logs:
        return

    update_position(read_data, label_suffix, t)
    update_raw_readings(read_data, label_suffix, t)
    update_baro_readings(read_data, label_suffix, t)
    update_motor_readings(read_data, label_suffix, t)
    update_battery_readings(read_data, label_suffix, t)
