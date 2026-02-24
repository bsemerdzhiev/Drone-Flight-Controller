use core::{fmt, time::Duration};

use tudelft_quadrupel::{
    barometer::read_pressure,
    battery::read_battery,
    block,
    motor::get_motors,
    mpu::{
        read_dmp_bytes, read_raw,
        structs::{Accel, Gyro, Quaternion},
    },
};

use crate::yaw_pitch_roll::YawPitchRoll;

pub struct Telemetry_Data {
    dt: Duration,
    motors: [u16; 4],
    quaternion: Quaternion,
    ypr: YawPitchRoll,
    accel: Accel,
    gyro: Gyro,
    bat: u16,
    pres: u32,
}

impl Telemetry_Data {
    pub fn read_telemetry_data(dt: Duration) -> Self {
        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, gyro) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();
        return Telemetry_Data {
            dt,
            motors,
            quaternion,
            ypr,
            accel,
            gyro,
            bat,
            pres,
        };
    }
}

impl fmt::Display for Telemetry_Data {
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
            self.dt.as_millis(),
            self.motors[0],
            self.motors[1],
            self.motors[2],
            self.motors[3],
            self.ypr.yaw,
            self.ypr.pitch,
            self.ypr.roll,
            self.accel.x,
            self.accel.y,
            self.accel.z,
            self.bat,
            self.pres
        )
    }
}
