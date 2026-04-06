import dearpygui.dearpygui as dpg

from set_up_ui.set_up_live_feed import set_up_live_feed
from set_up_ui.set_up_logged_feed import set_up_logged_feed
from set_up_ui.set_up_message_log import set_up_message_log
from set_up_ui.set_up_loop_timing import set_up_loop_timing
from set_up_ui.general_info import set_up_general_info

from util.states import SENSOR_NAMES
import util.data as stored_data
import numpy


def toggle_pause():
    stored_data.pause_logs ^= True

    if stored_data.pause_logs:
        dpg.bind_item_theme("pause_btn", red_theme)
    else:
        dpg.bind_item_theme("pause_btn", blue_theme)


def toggle_chosen_sensors():
    stored_data.chosen_sensors ^= True

    if stored_data.chosen_sensors:
        dpg.bind_item_theme("chosen_sensors", red_theme)
    else:
        dpg.bind_item_theme("chosen_sensors", blue_theme)

    stored_data.baro_var_calc = numpy.array([0.0])
    stored_data.accel_var_calc = numpy.array([0.0])

    dpg.set_value(
        "chosen_sensors_text", stored_data.sensor_names[stored_data.chosen_sensors]
    )

    dpg.configure_item(
        "chosen_sensors_text",
        color=SENSOR_NAMES.get(
            stored_data.sensor_names[stored_data.chosen_sensors], [255, 255, 255]
        ),
    )


def set_up_gui():
    dpg.create_context()

    global blue_theme, red_theme

    with dpg.theme() as red_theme:
        with dpg.theme_component(dpg.mvTabButton):
            dpg.add_theme_color(dpg.mvThemeCol_Tab, (180, 30, 30, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (210, 50, 50, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabActive, (240, 70, 70, 255))

    with dpg.theme() as blue_theme:
        with dpg.theme_component(dpg.mvTabButton):
            dpg.add_theme_color(dpg.mvThemeCol_Tab, (30, 30, 180, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (50, 50, 210, 255))
            dpg.add_theme_color(dpg.mvThemeCol_TabActive, (70, 70, 240, 255))

    with dpg.window(tag="main", width=1500, height=900):
        with dpg.tab_bar():
            set_up_general_info()
            set_up_live_feed()
            set_up_logged_feed()
            set_up_message_log()
            set_up_loop_timing()
            dpg.add_tab_button(
                label="Pause",
                tag="pause_btn",
                callback=lambda: toggle_pause(),
            )
            dpg.bind_item_theme("pause_btn", blue_theme)
            dpg.add_tab_button(
                label="Swap sensors",
                tag="chosen_sensors",
                callback=lambda: toggle_chosen_sensors(),
            )
            dpg.bind_item_theme("chosen_sensors", blue_theme)
