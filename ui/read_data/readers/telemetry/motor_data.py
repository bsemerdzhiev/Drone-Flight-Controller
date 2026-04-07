from read_data.util.log_message import log_message
import util.data as stored_data


def read_motor_data(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

    # Motors
    motor_read = t.get("motors")

    for i in range(4):
        to_add_to.motors[i].append(motor_read[i])
    log_message(
        "Drone>PC",
        f"[Motors] M0={t['motors'][0]} M1={t['motors'][1]} M2={t['motors'][2]} M3={t['motors'][3]} flash={t.get('logged_in_flash')}",
    )
