use crate::pc_command;
use crate::telemetry_data::TelemetryData;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct DebugRpms {
    rpms: [u16; 4],
}

impl DebugRpms {
    pub fn new(rpms: &[u16; 4]) -> Self {
        return Self {
            rpms: [rpms[0], rpms[1], rpms[2], rpms[3]],
        };
    }
}

#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
#[repr(u8)]
pub enum FSMState {
    SafeMode,
    PanicMode,
    ManualMode,
    CalibrationMode,
    YawControl,
    FullControlMode,
    RawSensorsFullControlMode,
    HeightControlMode,
    WirelessMode,
}

#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct DroneInfo {
    state: FSMState,
    bat_level: u16,
}

impl DroneInfo {
    pub fn new(state: FSMState, bat_level: u16) -> Self {
        DroneInfo { state, bat_level }
    }
}

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub enum DeviceCommand {
    DroneInfo(DroneInfo),
    ChangeMode(FSMState),
    ManualInput(pc_command::ManualInput),
    Telemetry(TelemetryData),
    Ack,
    DebugRpms(DebugRpms),
}
