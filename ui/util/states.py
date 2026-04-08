FSM_STATES = [
    "SafeMode",
    "PanicMode",
    "ManualMode",
    "CalibrationMode",
    "YawControlMode",
    "FullControlMode",
    "RawMode",
]

FSM_COLORS = {
    "SafeMode": [100, 100, 255],
    "PanicMode": [255, 60, 60],
    "ManualMode": [100, 220, 100],
    "CalibrationMode": [220, 180, 50],
    "YawControlMode": [80, 200, 200],
    "FullControlMode": [180, 100, 255],
    "RawMode": [200, 200, 200],
}

COM_MODES = {
    False: [100, 100, 255],
    True: [255, 60, 60],
}

COM_MODES_NAMES = {False: "UART", True: "BlueTooth"}

SENSOR_NAMES = {"DMP": [220, 180, 50], "Kalman(Simplified)": [100, 100, 255]}
