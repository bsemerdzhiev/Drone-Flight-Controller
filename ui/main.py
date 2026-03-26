import threading
import dearpygui.dearpygui as dpg

from read_data import serial_reader
from states import FSM_COLORS
from update_gui import update_gui
from set_up import set_up_gui


if __name__ == "__main__":
    # Start GUI
    # ---------------------------------------
    threading.Thread(target=serial_reader, daemon=True).start()
    # TODO:
    # get host screen viewport
    set_up_gui()

    # Launch threads
    # ---------------------------------------
    # serial_reader()
# threading.Thread(target=update_gui, daemon=True).start()
