import dearpygui.dearpygui as dpg

from states import FSM_COLORS


def set_up_gui():
    dpg.create_context()

    with dpg.window(
        label="Drone Ground Station", tag="main_window", width=900, height=800
    ):
        dpg.add_separator()

        # Two column layout
        with dpg.group(horizontal=True):
            # ---- LEFT COLUMN ----
            with dpg.child_window(width=560, height=940, border=False):
                # --- FSM State ---
                dpg.add_text("FSM State", color=[255, 255, 100])
                with dpg.group(horizontal=True):
                    dpg.add_text("Current State:", color=[180, 180, 180])
                    dpg.add_text(
                        "SafeMode", tag="fsm_display", color=FSM_COLORS["SafeMode"]
                    )

                dpg.add_separator()

                # --- Battery ---
                dpg.add_text("Battery Level", color=[255, 255, 100])
                dpg.add_progress_bar(
                    tag="battery_bar", default_value=1.0, width=400, overlay="100%"
                )
                dpg.add_text("100%", tag="battery_text")

                dpg.add_separator()

                # --- P values ---
                dpg.add_text("Controller P-Values", color=[255, 255, 100])
                with dpg.table(
                    header_row=True,
                    borders_innerH=True,
                    borders_outerH=True,
                    borders_innerV=True,
                    borders_outerV=True,
                ):
                    dpg.add_table_column(label="Controller")
                    dpg.add_table_column(label="P Value")
                    for axis in ["yaw", "pitch", "roll"]:
                        with dpg.table_row():
                            dpg.add_text(f"{axis.capitalize()} P:")
                            dpg.add_text("1.000", tag=f"p_{axis}_display")

                dpg.add_separator()

                # --- Joystick Inputs ---
                dpg.add_text("Joystick Inputs", color=[255, 255, 100])
                with dpg.table(
                    header_row=True,
                    borders_innerH=True,
                    borders_outerH=True,
                    borders_innerV=True,
                    borders_outerV=True,
                ):
                    dpg.add_table_column(label="Axis")
                    dpg.add_table_column(label="Value")
                    dpg.add_table_column(label="Bar")
                    for axis in ["pitch", "roll", "lift", "yaw"]:
                        with dpg.table_row():
                            dpg.add_text(axis.capitalize())
                            dpg.add_text("0.000", tag=f"{axis}_val")
                            dpg.add_progress_bar(
                                tag=f"{axis}_bar", default_value=0.5, width=200
                            )

                dpg.add_separator()

                # --- Motor Outputs ---
                dpg.add_text("Motor Outputs", color=[255, 255, 100])
                for i in range(4):
                    with dpg.group(horizontal=True):
                        dpg.add_text(f"M{i}:")
                        dpg.add_progress_bar(
                            tag=f"motor{i}", default_value=0, width=350
                        )
                        dpg.add_text("0", tag=f"motor{i}_val")

                dpg.add_separator()

                # --- Accel & Gyro ---
                dpg.add_text("IMU Data", color=[255, 255, 100])
                with dpg.table(
                    header_row=True,
                    borders_innerH=True,
                    borders_outerH=True,
                    borders_innerV=True,
                    borders_outerV=True,
                ):
                    dpg.add_table_column(label="Sensor")
                    dpg.add_table_column(label="X (i16)")
                    dpg.add_table_column(label="Y (i16)")
                    dpg.add_table_column(label="Z (i16)")

                    with dpg.table_row():
                        dpg.add_text("Accel")
                        dpg.add_text("0", tag="accel_x")
                        dpg.add_text("0", tag="accel_y")
                        dpg.add_text("0", tag="accel_z")

                    with dpg.table_row():
                        dpg.add_text("Gyro")
                        dpg.add_text("0", tag="gyro_x")
                        dpg.add_text("0", tag="gyro_y")
                        dpg.add_text("0", tag="gyro_z")

            # ---- RIGHT COLUMN ----
            with dpg.child_window(width=580, height=940, border=False):
                # --- Rate Plots ---
                dpg.add_text("Sensor Rates", color=[255, 255, 100])

                for label, tag_x, tag_y, series_tag in [
                    ("Yaw Rate", "x_axis_yaw", "y_axis_yaw", "yaw_series"),
                    ("Pitch Rate", "x_axis_pitch", "y_axis_pitch", "pitch_series"),
                    ("Roll Rate", "x_axis_roll", "y_axis_roll", "roll_series"),
                ]:
                    with dpg.plot(label=label, height=180, width=-1):
                        dpg.add_plot_axis(dpg.mvXAxis, label="time", tag=tag_x)
                        dpg.add_plot_axis(dpg.mvYAxis, label="deg/s", tag=tag_y)
                        dpg.add_line_series([], [], parent=tag_y, tag=series_tag)

                dpg.add_separator()

                # --- Message Log ---
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
                    height=280,
                ):
                    dpg.add_table_column(
                        label="Time", width_fixed=True, init_width_or_weight=70
                    )
                    dpg.add_table_column(
                        label="Direction", width_fixed=True, init_width_or_weight=90
                    )
                    dpg.add_table_column(label="Message")

    dpg.create_viewport(title="Drone UI", width=1920, height=1080)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()
    dpg.destroy_context()
