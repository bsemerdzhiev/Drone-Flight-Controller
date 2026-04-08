from read_data.util.log_message import log_message
import util.data as stored_data


def read_manual_input(t):
    log_message(
        "PC>Drone",
        f"ManualInput lift={t['lift']:.3f} roll={t['roll']:.3f} "
        f"pitch={t['pitch']:.3f} yaw={t['yaw']:.3f}",
    )

    for space_pos in [
        "pitch",
        "yaw",
        "roll",
        "lift",
        "yaw_p_trim",
        "roll_pitch_p_trim",
        "roll_pitch_d_trim",
    ]:
        stored_data.joystick[space_pos].append(t[space_pos])
    stored_data.telemetry_data_size = t["telemetry_data_size"]
