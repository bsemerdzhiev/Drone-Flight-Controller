use core::time::Duration;

use crate::yaw_pitch_roll::YawPitchRoll;
use my_hdlc::telemetry_data::TelemetryData;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw, structs::*};

pub trait TelemetryRead {
    fn read_telemetry(dt: Duration) -> Self;
}

impl TelemetryRead for TelemetryData {
    fn read_telemetry(dt: Duration) -> Self {
        // let motors = get_motors();
        // let quaternion = read_dmp_bytes().unwrap();
        // let ypr = YawPitchRoll::from(quaternion);
        // let (accel_raw, gyro_raw) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();

        return TelemetryData {
            dt: dt.as_millis() as u128,
            // motors,
            // yaw: ypr.yaw,
            // pitch: ypr.pitch,
            // roll: ypr.roll,
            // accel_x: accel_raw.x,
            // accel_y: accel_raw.y,
            // accel_z: accel_raw.z,
            // gyro_x: gyro_raw.x,
            // gyro_y: gyro_raw.y,
            // gyro_z: gyro_raw.z,
            bat,
            pres,
        };
    }
}
