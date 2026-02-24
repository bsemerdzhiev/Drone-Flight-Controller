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
pub enum CommandType {
    ChangeMode,
    OtherMode,
}

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub struct DroneControl {
    desired_lift: f32,
    roll_rate: f32,
    pitch_rate: f32,
    yaw_rate: f32,
}

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub enum Command {
    SendDroneControl(DroneControl),
    ChangeFSMState(FSMState),
}
