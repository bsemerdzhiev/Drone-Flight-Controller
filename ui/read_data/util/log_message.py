from util import data as stored_data
import time


def log_message(direction: str, msg: str):
    """direction: 'PC>Drone' or 'Drone>PC'"""
    ts = time.strftime("%H:%M:%S")
    stored_data.message_log.append((ts, direction, msg))
