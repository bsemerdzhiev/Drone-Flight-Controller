use crate::pc_command::{self, ManualDroneInput, ManualDroneTrims};
use crate::telemetry_data::TelemetryData;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct DebugRpms {
    pub rpms: [i32; 4],
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct DebugYawPitchRoll {
    pub info: [f32; 5],
}

#[derive(Serialize, Deserialize, Debug, PartialEq)]
pub struct DebugCalibration {
    pub ypr_offset: [f32; 3],
}

impl DebugRpms {
    pub fn new(rpms: &[i32; 4]) -> Self {
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

    pub fn state(&self) -> FSMState {
        self.state
    }

    pub fn bat_level(&self) -> u16 {
        self.bat_level
    }
}

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub enum DeviceCommand {
    DroneInfo(DroneInfo),
    ChangeMode(FSMState),
    ManualInput(ManualDroneInput),
    ManualDroneTrims(ManualDroneTrims),
    Telemetry(TelemetryData),
    Ack,
    DebugRpms(DebugRpms),
    DebugYawPitchRoll(DebugYawPitchRoll),
    DebugCalibration(DebugCalibration),
}
