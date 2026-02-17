use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, PartialEq)]
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

#[derive(Serialize, Deserialize, PartialEq)]
#[repr(u8)]
pub enum CommandType {
    ChangeMode,
}

#[derive(Serialize, Deserialize, PartialEq)]
pub struct Command {
    command_type: CommandType,
    fsm_state: Option<FSMState>,
}

impl Command {
    pub fn new(command_type: CommandType, fsm_state: Option<FSMState>) -> Self {
        Self {
            command_type,
            fsm_state,
        }
    }
}
