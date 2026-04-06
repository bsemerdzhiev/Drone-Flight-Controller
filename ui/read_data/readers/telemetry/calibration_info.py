from read_data.util.log_message import log_message
import util.data as stored_data


def read_calibration_info(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

    # Pressure Raw
    for chosen in [
        "averaged_accel",
        "averaged_gyro",
        "averaged_ypr",
    ]:
        to_add_to.calibration_data[chosen].append(t.get(chosen))

    log_message(
        "Drone>PC",
        f"[CalibrationInfo] accel={t.get('averaged_accel')}, gyro={t.get('averaged_gyro')}, ypr={t.get('averaged_ypr')} flash={t.get('logged_in_flash')}",
    )
