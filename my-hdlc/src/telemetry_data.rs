use core::fmt;
use serde::{Deserialize, Serialize};

use crate::command::FSMState;
// pub const TELEMETERY_DATA_SIZE: u32 = 2 * (32 + (4 * 16) + (3 * 32) + 16 + 32 + 4) + 2;
pub const TELEMETERY_DATA_SIZE: u32 = core::mem::size_of::<TelemetryData>() as u32;
#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct TelemetryData {
    pub logged_in_flash: bool,
    pub dt: u32,
    pub motors: [u16; 4],

    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,

    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,

    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,

    pub bat: u16,
    pub pres: u32,
    pub cur_state: FSMState,
}
