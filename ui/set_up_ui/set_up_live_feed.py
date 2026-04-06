import dearpygui.dearpygui as dpg

from set_up_ui.set_up_sensors import set_up_sensors


def set_up_live_feed():
    with dpg.tab(label="Drone Live Feed"):
        with dpg.child_window(height=-1, border=False):
            set_up_sensors("_live")
