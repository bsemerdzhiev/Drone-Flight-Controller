import json
import time
import dearpygui.dearpygui as dpg


def send_data(sock_file):
    while True:
        axes = ["yaw", "pitch", "roll", "lift"]
        terms = ["p", "i", "d"]

        pid_trims = {
            axis: {term: dpg.get_value(f"{axis}_{term}_trim") for term in terms}
            for axis in axes
        }
        # print(pid_trims, "\n\r")
        sock_file.write(json.dumps(pid_trims) + "\n")
        sock_file.flush()
        # pid_trims["yaw"]["p"], pid_trims["roll"]["d"], etc.

        time.sleep(3)
