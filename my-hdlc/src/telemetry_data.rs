use core::fmt;
use serde::{Deserialize, Serialize};

use crate::command::FSMState;
// pub const TELEMETERY_DATA_SIZE: u32 = 2 * (32 + (4 * 16) + (3 * 32) + 16 + 32 + 4) + 2;
pub const TELEMETERY_DATA_SIZE: u32 = core::mem::size_of::<TelemetryData>() as u32;
#[repr(C)]
#[derive(Serialize, Deserialize, PartialEq, Debug, Clone, Copy)]
pub struct TelemetryData {
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
    pub p_yaw: f32,
    pub p_pitch: f32,
    pub p_roll: f32,
}

// impl fmt::Display for TelemetryData {
//     fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//         write!(
//             f,
//             "DTT: {:?}ms\n
//             MTR: {} {} {} {}\n
//             YPR {} {} {}\n
//             ACC {} {} {}\n
//             BAT {}\n
//             BAR {}\n",
//             // "DTT: {:?}ms\n
//             // MTR: {} {} {} {}\n
//             // YPR {} {} {}\n
//             // BAT {}\n
//             // BAR {}\n",
//             // "
//             // DTT: {:?}ms\n
//             // BAT {}\n
//             // BAR {}\n",
//             self.dt,
//             self.motors[0],
//             self.motors[1],
//             self.motors[2],
//             self.motors[3],
//             // self.yaw,
//             // self.pitch,
//             // self.roll,
//             self.accel_x,
//             self.accel_y,
//             self.accel_z,
//             self.bat,
//             self.pres
//         )
//     }
// }
