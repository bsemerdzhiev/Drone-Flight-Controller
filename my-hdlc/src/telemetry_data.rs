use core::fmt;
use serde::{Deserialize, Serialize};
#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub struct TelemetryData {
    pub dt: u128,
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
}

impl fmt::Display for TelemetryData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // write!(
        //     f,
        //     "x: {}, y: {}, steps: {}",
        //     (self.pos_x as i16 - self.starting_x as i16),
        //     (self.pos_y as i16 - self.starting_y as i16),
        //     self.step_count
        // )
        write!(
            f,
            "DTT: {:?}ms\n
        MTR: {} {} {} {}\n
        YPR {} {} {}\n
        ACC {} {} {}\n
        BAT {}\n
        BAR {}\n",
            self.dt,
            self.motors[0],
            self.motors[1],
            self.motors[2],
            self.motors[3],
            self.yaw,
            self.pitch,
            self.roll,
            self.accel_x,
            self.accel_y,
            self.accel_z,
            self.bat,
            self.pres
        )
    }
}
