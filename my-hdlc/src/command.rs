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
    rpms: [i32; 4],
}

impl DebugRpms {
    pub fn new(rpms: &[i32; 4]) -> Self {
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
