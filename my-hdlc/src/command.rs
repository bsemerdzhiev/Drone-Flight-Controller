use crate::pc_command;
use crate::telemetry_data::TelemetryData;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq, Debug)]
#[repr(u8)]
pub enum CommandType {
    ChangeMode,
    OtherMode,
}

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

// #[derive(Serialize, Deserialize, PartialEq, Debug)]
// pub enum Command {
// ManualInput(pc_command::ManualInput),
// ChangeFSMState(FSMState),
// DebugRpms(DebugRpms),
// }

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
}

impl DroneInfo {
    pub fn new(state: FSMState) -> Self {
        DroneInfo { state }
    }
}

#[derive(Serialize, Deserialize, PartialEq, Debug)]
#[repr(u8)]
pub enum DeviceCommand {
    DroneInfo(DroneInfo),
    ChangeMode(FSMState),
    ManualInput(pc_command::ManualInput),
    Telemetry(TelemetryData),
    Ack,
    DebugRpms(DebugRpms),
}

// impl DeviceCommand {
//     pub fn new(command_type: CommandType, fsm_state: Option<FSMState>) -> Self {
//         Self {
//             command_type,
//             fsm_state,
//         }
//     }

//     pub fn get_command_type(&self) -> &CommandType {
//         return &self.command_type;
//     }
//     pub fn get_fsm_state(&self) -> &Option<FSMState> {
//         return &self.fsm_state;
//     }
// }
