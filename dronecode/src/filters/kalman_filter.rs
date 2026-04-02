use core::ops::Sub;
use core::time::Duration;

use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::{CalibrationState, LSB_FOR_ACCEL};
use crate::util::approx_funcs::{approx_atan2, approx_sqrt};
use crate::util::axis::Axis;
use crate::util::yaw_pitch_roll::*;
use fixed::types::{I16F16, I26F6, I2F30, I30F2, I32F0, I4F28, I5F27};
use libm::{atan2f, sqrtf};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::mpu::{read_raw, structs::*};
use tudelft_quadrupel::time::Instant;

const C1: I4F28 = I4F28::lit("1.4");
const C2: I4F28 = I4F28::lit("1e-5");

const LSB_ACCEL_TO_GS: I4F28 = I4F28::lit("16384");

pub struct KalmanFilter {
    bias_p: DegreeType,
    bias_q: DegreeType,
    reading: (Axis<I4F28>, Axis<I4F28>),
    last_read_time: Instant,

    pub roll: I4F28,
    pub pitch: I4F28,
    pub yaw: I26F6,

    pub calibration_offset: (Axis<I32F0>, Axis<I32F0>),
}

impl KalmanFilter {
    pub fn new(offset: (Axis<I32F0>, Axis<I32F0>)) -> Self {
        KalmanFilter {
            bias_p: DegreeType::from_num(0),
            bias_q: DegreeType::from_num(0),
            reading: (Axis::<I4F28>::default(), Axis::<I4F28>::default()),
            last_read_time: Instant::now(),

            roll: I4F28::from_num(0),
            pitch: I4F28::from_num(0),

            yaw: I26F6::from_num(0),

            calibration_offset: offset,
        }
    }
    fn update_roll(&mut self, dt: DegreeType) {
        let p_clean = (self.reading.1.x) * DEGREE_TO_RAD - self.bias_p;
        // let raw_roll = atan2f(self.reading.0.y as f32, self.reading.0.z as f32);
        let raw_roll = approx_atan2(self.reading.0.y, self.reading.0.z);

        let estimated_roll = self.roll + p_clean * dt;
        let e = estimated_roll - raw_roll;
        self.roll = estimated_roll - e / C1;
        self.bias_p = self.bias_p + (e / dt) * C2;
    }
    fn update_pitch(&mut self, dt: DegreeType) {
        let ay = I30F2::from_num(self.reading.0.y);
        let az = I30F2::from_num(self.reading.0.z);

        let q_clean: DegreeType =
            (DegreeType::from_num(self.reading.1.y) * DEGREE_TO_RAD) - self.bias_q;

        let raw_pitch: DegreeType = DegreeType::from_num(approx_atan2(
            I30F2::from_num(-self.reading.0.x),
            approx_sqrt(ay * ay + az * az),
        ));

        let estimated_pitch = self.pitch + q_clean * dt;
        let e = estimated_pitch - raw_pitch;
        self.pitch = estimated_pitch - e / C1;

        self.bias_q = self.bias_q + (e / dt) * C2;
    }

    fn update_yaw(&mut self, dt: DegreeType) {
        //NOTE: only if we want to get absolute yaw
        // let dt = Instant::now()
        //     .duration_since(self.last_read_time)
        //     .as_secs_f32();
        //
        // self.yaw += self.reading.1.z * dt;

        // get the rate instead
        self.yaw = DegreeType::from_num(self.reading.1.z) * dt;
    }
}

impl ImuHandler for KalmanFilter {
    fn append_new_reading(&mut self) {
        let raw_read = read_raw().unwrap();

        let parsed_raw_read = (
            Axis::<I32F0>::from(raw_read.0),
            Axis::<I32F0>::from(raw_read.1),
        );

        self.reading.0.x = I4F28::from_num(
            parsed_raw_read
                .0
                .x
                .saturating_sub(self.calibration_offset.0.x),
        ) / LSB_ACCEL_TO_GS;
        self.reading.0.y = I4F28::from_num(
            parsed_raw_read
                .0
                .y
                .saturating_sub(self.calibration_offset.0.y),
        ) / LSB_ACCEL_TO_GS;
        self.reading.0.z = I4F28::from_num(
            parsed_raw_read
                .0
                .z
                .saturating_sub(self.calibration_offset.0.z),
        ) / LSB_ACCEL_TO_GS;

        self.reading.1.x = parsed_raw_read
            .1
            .x
            .saturating_sub(self.calibration_offset.1.x);
        self.reading.1.y = parsed_raw_read
            .1
            .y
            .saturating_sub(self.calibration_offset.1.y);
        self.reading.1.z = parsed_raw_read
            .1
            .z
            .saturating_sub(self.calibration_offset.1.z);

        let cur_time = Instant::now();
        let dt: DegreeType = DegreeType::from_num(
            cur_time
                .duration_since(self.last_read_time)
                .as_secs_f32()
                .clamp(0.001, 0.03),
        );

        self.update_pitch(dt);
        self.update_roll(dt);
        self.update_yaw(dt);

        self.last_read_time = cur_time;
    }
    fn get_reading<T, Y>(&mut self) -> YawPitchRoll<T, Y> {
        YawPitchRoll::<T, Y> {
            lift: T::from_num(0),
            yaw: T::from_num(self.yaw),
            pitch: T::from_num(-self.pitch * RAD_TO_DEGREE),
            roll: T::from_num(self.roll * RAD_TO_DEGREE),

            pressure: Y::from_num(0),
        }
    }
}
