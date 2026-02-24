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

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub enum Command {
    ManualInput(pc_command::ManualInput),
    ChangeFSMState(FSMState),
}
