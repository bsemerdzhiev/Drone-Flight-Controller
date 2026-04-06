import numpy as np
import dearpygui.dearpygui as dpg


def fit_with_margin(y_axis_tag: str, data: list, margin: float = 0.2):
    if not data:
        return
    lo, hi = min(data), max(data)
    span = hi - lo
    if span == 0:
        span = abs(lo) if lo != 0 else 1.0
    pad = span * margin

    dpg.set_axis_limits(y_axis_tag, lo - pad, hi + pad)


def update_loop_timing(read_data, label_suffix):
    t = list(read_data.time_data)
    loop_times = list(read_data.general_data["time_for_main_loop"])

    dpg.set_value("loop_ts_series" + label_suffix, [t, loop_times])
    dpg.fit_axis_data("x_axis_loop_ts" + label_suffix)

    fit_with_margin("y_axis_loop_ts" + label_suffix, loop_times)

    if loop_times:
        counts, edges = np.histogram(loop_times, bins=30)
        centers = ((np.array(edges[:-1]) + np.array(edges[1:])) / 2).tolist()
        dpg.set_value("loop_hist_series" + label_suffix, [centers, counts.tolist()])
        dpg.fit_axis_data("x_axis_loop_hist" + label_suffix)
    dpg.set_axis_limits("y_axis_loop_hist" + label_suffix, 0, len(loop_times))
