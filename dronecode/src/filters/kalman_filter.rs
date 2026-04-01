use core::ops::Sub;
use core::time::Duration;

use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::util::axis::Axis;
use crate::util::yaw_pitch_roll::*;
use libm::{atan2f, sqrtf};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::mpu::{read_raw, structs::*};
use tudelft_quadrupel::time::Instant;

const C1: f32 = 2.5f32;
const C2: f32 = 1000f32;

pub struct KalmanFilter {
    bias_p: f32,
    bias_q: f32,
    reading: (Axis<i32>, Axis<i32>),
    last_read_time: Instant,
    pub roll: f32,
    pub pitch: f32,
    yaw: f32,

    pub calibration_offset: (Axis<i32>, Axis<i32>),
}

impl KalmanFilter {
    pub fn new(offset: (Axis<i32>, Axis<i32>)) -> Self {
        KalmanFilter {
            bias_p: 0.0,
            bias_q: 0.0,
            reading: (
                Axis::<i32> { x: 0, y: 0, z: 0 },
                Axis::<i32> { x: 0, y: 0, z: 0 },
            ),
            last_read_time: Instant::now(),
            roll: 0f32,
            pitch: 0f32,
            yaw: 0f32,

            calibration_offset: offset,
        }
    }
    fn update_roll(&mut self, dt: f32) {
        let DEG_TO_RAD: f32 = micromath::F32Ext::acos(-1.0) / 180.0;

        let p_clean = (self.reading.1.x as f32 * DEG_TO_RAD) - self.bias_p;
        let raw_roll = atan2f(self.reading.0.y as f32, self.reading.0.z as f32);

        let estimated_roll = self.roll + p_clean * dt;
        let e = estimated_roll - raw_roll;
        self.roll = estimated_roll - e / C1;
        self.bias_p = self.bias_p + (e / dt) / C2;
    }
    fn update_pitch(&mut self, dt: f32) {
        let DEG_TO_RAD: f32 = micromath::F32Ext::acos(-1.0) / 180.0;

        let ay = self.reading.0.y as f32;
        let az = self.reading.0.z as f32;

        let q_clean = (self.reading.1.y as f32 * DEG_TO_RAD) - self.bias_q;
        let raw_pitch = atan2f(-self.reading.0.x as f32, sqrtf((ay * ay + az * az) as f32));

        let estimated_pitch = self.pitch + q_clean * dt;
        let e = estimated_pitch - raw_pitch;
        self.pitch = estimated_pitch - e / C1;

        self.bias_q = self.bias_q + (e / dt) / C2;
    }

    fn update_yaw(&mut self, dt: f32) {
        //NOTE: only if we want to get absolute yaw
        // let dt = Instant::now()
        //     .duration_since(self.last_read_time)
        //     .as_secs_f32();
        //
        // self.yaw += self.reading.1.z * dt;

        // get the rate instead
        self.yaw = self.reading.1.z as f32 * dt;
    }
}

impl ImuHandler for KalmanFilter {
    fn append_new_reading(&mut self) {
        let raw_read = read_raw().unwrap();

        self.reading = (Axis::from(raw_read.0), Axis::from(raw_read.1));

        self.reading.0.x = self.reading.0.x.saturating_sub(self.calibration_offset.0.x);
        self.reading.0.y = self.reading.0.y.saturating_sub(self.calibration_offset.0.y);
        self.reading.0.z = self.reading.0.z.saturating_sub(self.calibration_offset.0.z);

        self.reading.1.x = self.reading.1.x.saturating_sub(self.calibration_offset.1.x);
        self.reading.1.y = self.reading.1.y.saturating_sub(self.calibration_offset.1.y);
        self.reading.1.z = self.reading.1.z.saturating_sub(self.calibration_offset.1.z);

        let cur_time = Instant::now();
        let dt = cur_time
            .duration_since(self.last_read_time)
            .as_secs_f32()
            .clamp(0.001, 0.03);

        self.update_pitch(dt);
        self.update_roll(dt);
        self.update_yaw(dt);

        self.last_read_time = cur_time;
    }
    fn get_reading(&mut self) -> YawPitchRoll {
        let PI: f32 = micromath::F32Ext::acos(-1.0);
        let TO_DEGREES: f32 = 180.0 / PI;

        YawPitchRoll {
            lift: 0f32,
            yaw: self.yaw,
            pitch: -self.pitch * TO_DEGREES,
            roll: self.roll * TO_DEGREES,
            pressure: 0f32,
        }
    }
}
