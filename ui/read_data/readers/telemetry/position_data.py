from read_data.util.log_message import log_message
import util.data as stored_data
import math


def read_position_data(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

    log_message(
        "Drone>PC",
        f"[Position] yaw={math.degrees(t.get('yaw', 0.0)):.1f}° pitch={math.degrees(t.get('pitch', 0.0)):.1f}° roll={math.degrees(t.get('roll', 0.0)):.1f}° | kalman yaw={t.get('yaw_kalman', 0):.1f} pitch={t.get('pitch_kalman', 0):.1f} roll={t.get('roll_kalman', 0):.1f} flash={t.get('logged_in_flash')}",
    )

    # DMP Data
    to_add_to.yaw_data.append(math.degrees(t.get("yaw", 0.0)))
    to_add_to.pitch_data.append(math.degrees(t.get("pitch", 0.0)))
    to_add_to.roll_data.append(math.degrees(t.get("roll", 0.0)))

    # Kalman data
    to_add_to.yaw_kalman.append(t.get("yaw_kalman", 0.0))
    to_add_to.pitch_kalman.append(t.get("pitch_kalman", 0.0))
    to_add_to.roll_kalman.append(t.get("roll_kalman", 0.0))
