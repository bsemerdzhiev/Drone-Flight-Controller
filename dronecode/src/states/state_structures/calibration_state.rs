use core::{ops::Add, time::Duration};

use fixed::{
    traits::{Fixed, FixedSigned},
    types::{I16F16, I20F12, I26F6, I2F30, I4F28},
};
use tudelft_quadrupel::{
    barometer::read_pressure,
    block,
    fixed::types::{I32F0, I64F0},
    mpu::{
        read_dmp_bytes, read_raw,
        structs::{Accel, Gyro},
    },
    time::Instant,
};

use crate::util::{axis::Axis, yaw_pitch_roll::YawPitchRoll};

// since the accelerometer uses +-2G(16'384 LSB/g),
// this number needs to be subtracted from the calibrated
// Z axis
// https://www.alldatasheet.com/datasheet-pdf/download/1132807/TDK/MPU-6050.html
pub const LSB_FOR_ACCEL: i32 = 16384;

const CALIBRATION_TIME: Duration = Duration::from_secs(5);

#[derive(Copy, Clone, Debug)]
pub struct CalibrationState<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
{
    accelerometer_sum: Axis<I64F0>,
    gyro_sum: Axis<I64F0>,

    sample_cnt: I20F12,

    pub start_time: Instant,

    pub accelerometer_offset: Axis<I16F16>,
    pub gyro_offset: Axis<I16F16>,

    ypr_sum: Axis<I20F12>,
    pub ypr_offset: YawPitchRoll<T, Y>,
}

impl<T, Y> CalibrationState<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
{
    pub fn new() -> Self {
        Self {
            accelerometer_sum: Axis::<I64F0>::default(),
            gyro_sum: Axis::<I64F0>::default(),

            ypr_sum: Axis::<I20F12>::default(),
            sample_cnt: I20F12::from_num(0),

            start_time: Instant::now(),

            accelerometer_offset: Axis::<I16F16>::default(),
            gyro_offset: Axis::<I16F16>::default(),

            ypr_offset: YawPitchRoll::<T, Y>::new(),
        }
    }
    pub fn reset(&mut self) {
        *self = CalibrationState::new();
    }

    pub fn read_new_sample(&mut self) {
        let ypr_sample = YawPitchRoll::<T, Y>::from(block!(read_dmp_bytes()).unwrap());

        let raw_read = read_raw().unwrap();

        self.accelerometer_sum = self.accelerometer_sum + raw_read.0;
        self.gyro_sum = self.gyro_sum + raw_read.1;

        // self.ypr_sum = self.ypr_sum + ypr_sample;
        self.ypr_sum = Axis::<I20F12> {
            x: self.ypr_sum.x + I20F12::from_num(ypr_sample.yaw),
            y: self.ypr_sum.y + I20F12::from_num(ypr_sample.pitch),
            z: self.ypr_sum.z + I20F12::from_num(ypr_sample.roll),
        };

        self.sample_cnt += I20F12::from_num(1);
    }

    pub fn finalize_calibration(&mut self) {
        self.accelerometer_offset = Axis {
            x: I16F16::from_num(self.accelerometer_sum.x / I64F0::from_num(self.sample_cnt)),
            y: I16F16::from_num(self.accelerometer_sum.y / I64F0::from_num(self.sample_cnt)),
            z: I16F16::from_num(self.accelerometer_sum.z / I64F0::from_num(self.sample_cnt)),
        };
        self.accelerometer_offset.z -= I16F16::from_num(LSB_FOR_ACCEL);

        self.gyro_offset = Axis {
            x: I16F16::from_num(self.gyro_sum.x / I64F0::from_num(self.sample_cnt)),
            y: I16F16::from_num(self.gyro_sum.y / I64F0::from_num(self.sample_cnt)),
            z: I16F16::from_num(self.gyro_sum.z / I64F0::from_num(self.sample_cnt)),
        };

        self.ypr_offset = YawPitchRoll::<T, Y> {
            lift: T::from_num(0),

            yaw: T::from_num(self.ypr_sum.x / self.sample_cnt),
            pitch: T::from_num(self.ypr_sum.y / self.sample_cnt),
            roll: T::from_num(self.ypr_sum.z / self.sample_cnt),

            pressure: Y::from_num(0),
        };
    }
    pub fn should_finish(&mut self) -> bool {
        return Instant::now().duration_since(self.start_time) >= CALIBRATION_TIME;
    }
}
