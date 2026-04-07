import dearpygui.dearpygui as dpg
import json

PID_AXES = ["yaw", "pitch", "roll", "lift"]
PID_TERMS = ["p", "i", "d"]
PID_FILE = "pid_trims.json"


def make_range_callback(slider_tag, min_tag, max_tag):
    def cb(sender, app_data):
        lo = dpg.get_value(min_tag)
        hi = dpg.get_value(max_tag)
        dpg.configure_item(slider_tag, min_value=lo, max_value=hi)

    return cb


def save_pid_trims(sender, app_data):
    path = app_data["file_path_name"]
    data = {
        axis: {
            term: {
                "value": dpg.get_value(f"{axis}_{term}_trim"),
                "min": dpg.get_value(f"{axis}_{term}_min"),
                "max": dpg.get_value(f"{axis}_{term}_max"),
            }
            for term in PID_TERMS
        }
        for axis in PID_AXES
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)


def load_pid_trims(sender, app_data):
    path = app_data["file_path_name"]
    try:
        with open(path, "r") as f:
            data = json.load(f)
        for axis in PID_AXES:
            for term in PID_TERMS:
                entry = data[axis][term]
                lo, hi = entry["min"], entry["max"]
                dpg.set_value(f"{axis}_{term}_min", lo)
                dpg.set_value(f"{axis}_{term}_max", hi)
                dpg.configure_item(f"{axis}_{term}_trim", min_value=lo, max_value=hi)
                dpg.set_value(f"{axis}_{term}_trim", entry["value"])
    except (FileNotFoundError, KeyError):
        pass


def set_up_pid():
    with dpg.group(horizontal=True):
        with dpg.group(horizontal=True):
            dpg.add_button(
                label="Save PID Trims",
                callback=lambda: dpg.show_item("save_pid_dialog"),
            )
            dpg.add_button(
                label="Load PID Trims",
                callback=lambda: dpg.show_item("load_pid_dialog"),
            )
        with dpg.group():
            dpg.add_text("PID Trim Values", color=[255, 255, 100])
            with dpg.table(
                header_row=True,
                borders_innerV=True,
                borders_outerV=True,
                borders_innerH=True,
                borders_outerH=True,
            ):
                dpg.add_table_column(label="Axis")
                dpg.add_table_column(label="P")
                dpg.add_table_column(label="I")
                dpg.add_table_column(label="D")

                for axis in ["Yaw", "Pitch", "Roll", "Lift"]:
                    tag = axis.lower()
                    with dpg.table_row():
                        dpg.add_text(axis)
                        for term in ["p", "i", "d"]:
                            with dpg.group(horizontal=True):
                                dpg.add_input_float(
                                    tag=f"{tag}_{term}_min",
                                    default_value=-1.0,
                                    width=55,
                                    format="%.2f",
                                    step=0,
                                )
                                dpg.add_slider_float(
                                    tag=f"{tag}_{term}_trim",
                                    default_value=0.0,
                                    min_value=-1.0,
                                    max_value=1.0,
                                    width=120,
                                    format="%.3f",
                                    callback=make_range_callback(
                                        f"{tag}_{term}_trim",
                                        f"{tag}_{term}_min",
                                        f"{tag}_{term}_max",
                                    ),
                                )
                                dpg.add_input_float(
                                    tag=f"{tag}_{term}_max",
                                    default_value=1.0,
                                    width=55,
                                    format="%.2f",
                                    step=0,
                                )
    dpg.add_separator()
