use core::ops::Sub;
use core::time::Duration;

use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::{CalibrationState, LSB_FOR_ACCEL};
use crate::util::approx_funcs::{approx_sqrt, atan2_approx, atan2_cordic};
use crate::util::axis::Axis;
use crate::util::yaw_pitch_roll::*;
use fixed::traits::{Fixed, FixedSigned};
use fixed::types::I16F16;
use libm::{atan2f, sqrtf};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::mpu::{read_raw, structs::*};
use tudelft_quadrupel::time::Instant;

const C1: I16F16 = I16F16::lit("1.5");
const C2: I16F16 = I16F16::lit("1e-5");

const ACCEL_SAMPLE_RATE: Duration = Duration::from_millis(1);

const LSB_ACCEL_TO_GS: I16F16 = I16F16::lit("16384");
const RAD_TO_DEGREE: I16F16 = I16F16::lit("57.2957");
const DEGREE_TO_RAD: I16F16 = I16F16::lit("0.0174");

pub struct KalmanFilter {
    bias_p: I16F16,
    bias_q: I16F16,
    reading: (Axis<I16F16>, Axis<I16F16>),
    last_read_time: Instant,

    pub roll: I16F16,
    pub pitch: I16F16,
    pub yaw: I16F16,

    pub calibration_offset: (Axis<I16F16>, Axis<I16F16>),
}

impl KalmanFilter {
    pub fn new(offset: (Axis<I16F16>, Axis<I16F16>)) -> Self {
        KalmanFilter {
            bias_p: I16F16::from_num(0),
            bias_q: I16F16::from_num(0),
            reading: (Axis::<I16F16>::default(), Axis::<I16F16>::default()),
            last_read_time: Instant::now(),

            roll: I16F16::from_num(0),
            pitch: I16F16::from_num(0),

            yaw: I16F16::from_num(0),

            calibration_offset: offset,
        }
    }
    fn update_roll(&mut self, dt: I16F16) {
        let p_clean = (self.reading.1.x) * DEGREE_TO_RAD - self.bias_p;
        // let raw_roll = atan2f(self.reading.0.y as f32, self.reading.0.z as f32);
        let raw_roll = atan2_cordic(self.reading.0.y, self.reading.0.z);

        let estimated_roll = self.roll + p_clean * dt;
        let e = estimated_roll - raw_roll;
        self.roll = estimated_roll - e / C1;
        self.bias_p = self.bias_p + (e / dt) * C2;
    }
    fn update_pitch(&mut self, dt: I16F16) {
        let ay = self.reading.0.y;
        let az = self.reading.0.z;

        let q_clean: I16F16 = (self.reading.1.y * DEGREE_TO_RAD) - self.bias_q;

        let raw_pitch: I16F16 = I16F16::from_num(atan2_cordic(
            -self.reading.0.x,
            approx_sqrt(ay * ay + az * az),
        ));

        let estimated_pitch = self.pitch + q_clean * dt;
        let e = estimated_pitch - raw_pitch;
        self.pitch = estimated_pitch - e / C1;

        self.bias_q = self.bias_q + (e / dt) * C2;
    }

    fn update_yaw(&mut self, dt: I16F16) {
        //NOTE: only if we want to get absolute yaw
        // let dt = Instant::now()
        //     .duration_since(self.last_read_time)
        //     .as_secs_f32();
        //
        // self.yaw += self.reading.1.z * dt;

        // get the rate instead
        self.yaw = self.reading.1.z * dt;
    }
}

impl ImuHandler for KalmanFilter {
    fn append_new_reading(&mut self) {
        let cur_time = Instant::now();
        if cur_time.duration_since(self.last_read_time) < ACCEL_SAMPLE_RATE {
            return;
        }

        let raw_read = read_raw().unwrap();

        let parsed_raw_read = (
            Axis::<I16F16>::from(raw_read.0),
            Axis::<I16F16>::from(raw_read.1),
        );

        self.reading.0.x =
            I16F16::from_num(parsed_raw_read.0.x - self.calibration_offset.0.x) / LSB_ACCEL_TO_GS;
        self.reading.0.y =
            I16F16::from_num(parsed_raw_read.0.y - self.calibration_offset.0.y) / LSB_ACCEL_TO_GS;
        self.reading.0.z =
            I16F16::from_num(parsed_raw_read.0.z - self.calibration_offset.0.z) / LSB_ACCEL_TO_GS;

        self.reading.1.x = parsed_raw_read.1.x - self.calibration_offset.1.x;
        self.reading.1.y = parsed_raw_read.1.y - self.calibration_offset.1.y;
        self.reading.1.z = parsed_raw_read.1.z - self.calibration_offset.1.z;

        let dt: I16F16 =
            I16F16::from_num(cur_time.duration_since(self.last_read_time).as_millis()) / 1000;
        // .clamp(0.001, 0.03),

        self.update_pitch(dt);
        self.update_roll(dt);
        self.update_yaw(dt);

        self.last_read_time = cur_time;
    }
    fn get_reading<T, Y>(&mut self) -> YawPitchRoll<T, Y>
    where
        T: FixedSigned,
        Y: FixedSigned,
    {
        YawPitchRoll::<T, Y> {
            lift: T::from_num(0),
            yaw: T::from_num(self.yaw),
            pitch: T::from_num(-self.pitch * RAD_TO_DEGREE),
            roll: T::from_num(self.roll * RAD_TO_DEGREE),

            pressure: Y::from_num(0),
        }
    }
}
