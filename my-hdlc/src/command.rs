use serde::{Deserialize, Serialize};

use crate::pc_command;

#[derive(Serialize, Deserialize, PartialEq, Debug)]
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

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub enum Command {
    ManualInput(pc_command::ManualInput),
    ChangeFSMState(FSMState),
    DebugRpms(DebugRpms),
}
use crate::telemetry_data::TelemetryData;
use serde::{Deserialize, Serialize};
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

#[derive(Serialize, Deserialize, PartialEq, Debug)]
#[repr(u8)]
pub enum DeviceCommand {
    ChangeMode(FSMState),
    Telemetry(TelemetryData),
    Ack,
}

// #[derive(Serialize, Deserialize, PartialEq, Debug)]
// pub struct DeviceCommand {
//     command_type: CommandType,
//     fsm_state: Option<FSMState>,
// }

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
