import dearpygui.dearpygui as dpg


def set_up_loop_timing():
    with dpg.tab(label="Loop Timing"):
        with dpg.child_window(height=-1, border=False):
            for label_suffix in ["_live", "_logged"]:
                dpg.add_text(
                    "Live Timing" if label_suffix == "_live" else "Logged Timing",
                    color=[255, 255, 100],
                )

                with dpg.group(horizontal=True):
                    # Time series
                    with dpg.plot(label="Loop Time Series", height=250, width=430):
                        dpg.add_plot_axis(
                            dpg.mvXAxis,
                            label="time",
                            tag="x_axis_loop_ts" + label_suffix,
                        )
                        dpg.add_plot_axis(
                            dpg.mvYAxis, label="ms", tag="y_axis_loop_ts" + label_suffix
                        )
                        dpg.add_line_series(
                            [],
                            [],
                            label="loop time",
                            parent="y_axis_loop_ts" + label_suffix,
                            tag="loop_ts_series" + label_suffix,
                        )

                    # Histogram
                    with dpg.plot(label="Loop Time Histogram", height=250, width=430):
                        dpg.add_plot_axis(
                            dpg.mvXAxis,
                            label="ms",
                            tag="x_axis_loop_hist" + label_suffix,
                        )
                        dpg.add_plot_axis(
                            dpg.mvYAxis,
                            label="count",
                            tag="y_axis_loop_hist" + label_suffix,
                        )
                        dpg.add_bar_series(
                            [],
                            [],
                            label="distribution",
                            parent="y_axis_loop_hist" + label_suffix,
                            tag="loop_hist_series" + label_suffix,
                            weight=0.01,
                        )

                dpg.add_separator()
