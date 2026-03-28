use core::time::Duration;

use crate::filters::sensors_handler::ImuHandler;
use crate::util::yaw_pitch_roll::*;
use libm::{atan2f, sqrtf};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::mpu::{read_raw, structs::*};
use tudelft_quadrupel::time::Instant;

const C1: f32 = 50f32;
const C2: f32 = 100_000f32;

pub struct KalmanFilter {
    bias_p: f32,
    bias_q: f32,
    reading: (Accel, Gyro),
    last_read_time: Instant,
    roll: f32,
    pitch: f32,
    yaw: f32,

    calibration_offset: YawPitchRoll,
}

impl KalmanFilter {
    pub fn new(offset: YawPitchRoll) -> Self {
        KalmanFilter {
            bias_p: 0.0,
            bias_q: 0.0,
            reading: (Accel { x: 0, y: 0, z: 0 }, Gyro { x: 0, y: 0, z: 0 }),
            last_read_time: Instant::now(),
            roll: 0f32,
            pitch: 0f32,
            yaw: 0f32,

            calibration_offset: offset,
        }
    }
    fn update_roll(&mut self) {
        let p_clean = self.reading.1.x as f32 - self.bias_p;
        let raw_roll = atan2f(self.reading.0.y as f32, self.reading.0.z as f32);
        let cur_time = Instant::now();
        let dt = (cur_time.duration_since(self.last_read_time).as_millis() as f32) / 1000.0;

        let estimated_roll = self.roll + p_clean * dt;
        let e = estimated_roll - raw_roll;
        self.roll = estimated_roll - e / C1;
        self.bias_p = self.bias_p + (e / dt) / C2;
    }
    fn update_pitch(&mut self) {
        let q_clean = self.reading.1.y as f32 - self.bias_q;
        let raw_pitch = atan2f(
            -self.reading.0.x as f32,
            sqrtf(
                (self.reading.0.y * self.reading.0.y + self.reading.0.z * self.reading.0.z) as f32,
            ),
        );
        let cur_time = Instant::now();
        let dt = (cur_time.duration_since(self.last_read_time).as_millis() as f32) / 1000.0;

        let estimated_pitch = self.pitch + q_clean * dt;
        let e = estimated_pitch - raw_pitch;
        self.pitch = estimated_pitch - e / C1;
        self.bias_q = self.bias_q + (e / dt) / C2;
    }

    fn update_yaw(&mut self) {
        //NOTE: only if we want to get absolute yaw
        // let dt = Instant::now()
        //     .duration_since(self.last_read_time)
        //     .as_secs_f32();
        //
        // self.yaw += self.reading.1.z * dt;

        // get the rate instead
        self.yaw = self.reading.1.z as f32;
    }
}

impl ImuHandler for KalmanFilter {
    fn append_new_reading(&mut self, input: (Accel, Gyro)) {
        self.reading = input;

        self.update_pitch();
        self.update_roll();
        self.update_yaw();

        self.last_read_time = Instant::now();
    }
    fn get_reading(&mut self) -> Option<YawPitchRoll> {
        Some(YawPitchRoll {
            lift: 0f32,
            yaw: self.yaw,
            pitch: self.pitch,
            roll: self.roll,
            pressure: read_pressure() as f32,
        })
    }
}
