# Serial reading thread
# ---------------------------------------
import json
import socket
import time

from read_data.readers.manual_input import read_manual_input
from read_data.readers.telemetry.telemetry import read_telemetry

from read_data.readers.ble_info import read_ble_info
import util.data as stored_data


def serial_reader():
    global battery_level, fsm_state, joystick, p_values, accel, gyro

    SOCKET_PATH = "/tmp/drone_telemetry.sock"

    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    sock.connect(SOCKET_PATH)
    sock_file = sock.makefile("r")

    while True:
        try:
            line = sock_file.readline()
            if not line:
                continue

            t = json.loads(line)  # Data on JSON string
            # print(t)
            # print("\n\r")

            stored_data.received_packages.append(time.time())
            with stored_data.message_log_lock:
                if "Telemetry" in t:
                    t = t["Telemetry"]

                    read_telemetry(t)

                if "ManualInput" in t:
                    t = t["ManualInput"]

                    read_manual_input(t)

                if "BLEInfo" in t:
                    t = t["BLEInfo"]

                    read_ble_info(t)

        except Exception as e:
            print(f"Serial error: {e}")
