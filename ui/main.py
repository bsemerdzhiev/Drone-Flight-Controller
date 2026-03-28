import threading
import dearpygui.dearpygui as dpg

from read_data import serial_reader
from update_gui import update_step
from set_up import set_up_gui


if __name__ == "__main__":
    # Start GUI
    # ---------------------------------------
    set_up_gui()
    threading.Thread(target=serial_reader, daemon=True).start()
    # threading.Thread(target=update_gui, daemon=True).start()
    dpg.set_frame_callback(1, update_step)

    # TODO:
    # get host screen viewport
    dpg.create_viewport(title="Drone UI", width=1920, height=1080)
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.start_dearpygui()

    dpg.destroy_context()

    # Launch threads
    # ---------------------------------------
