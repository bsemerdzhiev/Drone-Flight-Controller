from read_data.readers.telemetry.general_data import read_general_data
from read_data.readers.telemetry.motor_data import read_motor_data
from read_data.readers.telemetry.calibration_info import read_calibration_info
from read_data.readers.telemetry.pid_info import read_pid_info
from read_data.readers.telemetry.position_data import read_position_data
from read_data.readers.telemetry.pressure_data import read_pressure_data
from read_data.readers.telemetry.raw_data import read_raw_data


def read_telemetry(t):
    if "GeneralData" in t:
        t = t["GeneralData"]

        read_general_data(t)

    if "MotorData" in t:
        t = t["MotorData"]

        read_motor_data(t)

    if "PositionData" in t:
        t = t["PositionData"]

        read_position_data(t)

    if "RawData" in t:
        t = t["RawData"]

        read_raw_data(t)

    if "PressureData" in t:
        t = t["PressureData"]

        read_pressure_data(t)

    if "PIDInfo" in t:
        t = t["PIDInfo"]

        read_pid_info(t)

    if "CalibrationInfo" in t:
        t = t["CalibrationInfo"]

        read_calibration_info(t)
