use core::time::Duration;

use crate::states::state_structures::state_context::LiveControllerValues;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use my_hdlc::command::FSMState;
use my_hdlc::telemetry_data::TelemetryData;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw, structs::*};

pub trait TelemetryRead {
    fn read_telemetry(dt: Duration, cur_state: FSMState, live_controller_values: &LiveControllerValues) -> Self;
}

impl TelemetryRead for TelemetryData {
    fn read_telemetry(dt: Duration, cur_state: FSMState, live_controller_values: &LiveControllerValues) -> Self {
        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes());
        let ypr = if quaternion.is_ok() {
            YawPitchRoll::from(quaternion.unwrap())
        } else {
            YawPitchRoll::new()
        };
        // let ypr = YawPitchRoll::from(quaternion);
        let (accel_raw, gyro_raw) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();
        return TelemetryData {
            dt: dt.as_millis() as u32,
            motors,
            // yaw: 0f32,
            // pitch: 0f32,
            // roll: 0f32,
            yaw: ypr.yaw,
            pitch: ypr.pitch,
            roll: ypr.roll,
            accel_x: accel_raw.x,
            accel_y: accel_raw.y,
            accel_z: accel_raw.z,
            gyro_x: gyro_raw.x,
            gyro_y: gyro_raw.y,
            gyro_z: gyro_raw.z,
            bat,
            pres,
            cur_state,
            p_yaw: live_controller_values.p_yaw,
            p_pitch: live_controller_values.p_pitch,
            p_roll: live_controller_values.p_roll,
        };
    }
}
