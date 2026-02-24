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
pub enum Command {
    ChangeMode(FSMState),
    Telemetry(TelemetryData),
}

// #[derive(Serialize, Deserialize, PartialEq, Debug)]
// pub struct Command {
//     command_type: CommandType,
//     fsm_state: Option<FSMState>,
// }

// impl Command {
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
