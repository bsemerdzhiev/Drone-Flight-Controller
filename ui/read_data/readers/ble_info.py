from read_data.util.log_message import log_message
import util.data as stored_data


def read_ble_info(t):
    stored_data.bluetooth["rssi"] = t["rssi"]

    log_message("PC>Drone", f"BLEInfo rssi={t['rssi']:.3f} ")
