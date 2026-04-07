from read_data.util.log_message import log_message
import util.data as stored_data


def read_general_data(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

        fsm_state = t.get("cur_state", "Unknown")

        stored_data.fsm_state = fsm_state

        stored_data.bluetooth["com_mode"] = t["com_mode"]

        stored_data.time_between_main_loop_runs.append(t["time_for_main_loop"])

    # Time
    if stored_data.start_time is None:
        stored_data.start_time = t["dt"]
    to_add_to.time_data.append(t["dt"] - stored_data.start_time)

    to_add_to.general_data["time_for_main_loop"].append(t["time_for_main_loop"] / 1000)

    # Battery Raw
    to_add_to.battery_level.append(t.get("bat", 0))
    log_message(
        "Drone>PC",
        f"[General] state={t.get('cur_state')} bat={t.get('bat')} time_for_main_loop={t.get('time_for_main_loop')}s, dt={t.get('dt')}s flash={t.get('logged_in_flash')}",
    )
