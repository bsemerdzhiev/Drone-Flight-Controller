from read_data.util.log_message import log_message
import util.data as stored_data


def read_pressure_data(t):
    to_add_to = stored_data.logged_data

    if not t["logged_in_flash"]:
        to_add_to = stored_data.live_data

    # Pressure Raw
    to_add_to.pres_data.append(t.get("pres", 0.0))
    to_add_to.pres_data_filtered.append(t.get("pressure_filtered", 0.0))

    log_message(
        "Drone>PC",
        f"[Pressure] raw={t.get('pres', 0.0):.2f} filtered={t.get('pressure_filtered', 0.0):.2f} flash={t.get('logged_in_flash')}",
    )
