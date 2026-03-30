use core::fmt;
use serde::{Deserialize, Serialize};

use crate::command::FSMState;
// pub const TELEMETERY_DATA_SIZE: u32 = 2 * (32 + (4 * 16) + (3 * 32) + 16 + 32 + 4) + 2;
pub const TELEMETERY_DATA_SIZE: usize = core::mem::size_of::<TelemetryData>();

#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct GeneralData {
    pub logged_in_flash: bool,
    pub dt: u32,

    pub bat: u16,
    pub cur_state: FSMState,
}

#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct MotorData {
    pub logged_in_flash: bool,
    pub motors: [u16; 4],
}

#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct PositionData {
    pub logged_in_flash: bool,

    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,

    pub yaw_kalman: f32,
    pub pitch_kalman: f32,
    pub roll_kalman: f32,
}

#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct RawData {
    pub logged_in_flash: bool,

    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,

    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
}

#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct PressureData {
    pub logged_in_flash: bool,

    pub pres: f32,
    pub pressure_filtered: f32,
}

#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub enum TelemetryData {
    GeneralData(GeneralData),
    MotorData(MotorData),
    PositionData(PositionData),
    RawData(RawData),
    PressureData(PressureData),
}
