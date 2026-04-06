from read_data.util.log_message import log_message
import util.data as stored_data


def read_pid_info(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

    # Pressure Raw
    to_add_to.pid_info["selected_height"].append(t.get("selected_height"))

    log_message(
        "Drone>PC",
        f"[PIDInfo] selected_height={t.get('selected_height', 0.0):.2f} flash={t.get('logged_in_flash')}",
    )
