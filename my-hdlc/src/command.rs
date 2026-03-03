use crate::telemetry_data::TelemetryData;
use serde::{Deserialize, Serialize};
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
