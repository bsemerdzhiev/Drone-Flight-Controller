from read_data.util.log_message import log_message
import util.data as stored_data

ACCEL_LSB = 16384
GYRO_LSB = 16.4


def read_raw_data(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

    log_message(
        "Drone>PC",
        f"[Raw] accel=({t.get('accel_x')},{t.get('accel_y')},{t.get('accel_z')}) gyro=({t.get('gyro_x')},{t.get('gyro_y')},{t.get('gyro_z')}) flash={t.get('logged_in_flash')}",
    )

    # Accellerometer Raw
    to_add_to.accel_raw["x"].append(t.get("accel_x", 0.0) / ACCEL_LSB)
    to_add_to.accel_raw["y"].append(t.get("accel_y", 0.0) / ACCEL_LSB)
    to_add_to.accel_raw["z"].append(t.get("accel_z", 0.0) / ACCEL_LSB)

    # Gyro Raw
    to_add_to.gyro_raw["x"].append(t.get("gyro_x", 0.0) / GYRO_LSB)
    to_add_to.gyro_raw["y"].append(t.get("gyro_y", 0.0) / GYRO_LSB)
    to_add_to.gyro_raw["z"].append(t.get("gyro_z", 0.0) / GYRO_LSB)
