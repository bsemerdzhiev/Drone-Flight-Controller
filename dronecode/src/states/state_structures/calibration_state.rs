use core::{ops::Add, time::Duration};

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

use crate::util::{axis::Axis, constants_file::ChosenFixedPointType, yaw_pitch_roll::YawPitchRoll};

// since the accelerometer uses +-2G(16'384 LSB/g),
// this number needs to be subtracted from the calibrated
// Z axis
// https://www.alldatasheet.com/datasheet-pdf/download/1132807/TDK/MPU-6050.html
pub const LSB_FOR_ACCEL: i32 = 16384;

const CALIBRATION_TIME: Duration = Duration::from_secs(5);

#[derive(Copy, Clone, Debug)]
pub struct CalibrationState {
    accelerometer_sum: Axis<I64F0>,
    gyro_sum: Axis<I64F0>,

    ypr_sum: YawPitchRoll,
    sample_cnt: I64F0,

    pub start_time: Instant,

    pub accelerometer_offset: Axis<I32F0>,
    pub gyro_offset: Axis<I32F0>,
    pub ypr_offset: YawPitchRoll,
}

impl CalibrationState {
    pub fn new() -> Self {
        Self {
            accelerometer_sum: Axis::<I64F0>::default(),
            gyro_sum: Axis::<I64F0>::default(),
            ypr_sum: YawPitchRoll::new(),
            sample_cnt: I64F0::from_num(0),
            start_time: Instant::now(),

            accelerometer_offset: Axis {
                x: I32F0::from_num(0),
                y: I32F0::from_num(0),
                z: I32F0::from_num(0),
            },
            gyro_offset: Axis {
                x: I32F0::from_num(0),
                y: I32F0::from_num(0),
                z: I32F0::from_num(0),
            },

            ypr_offset: YawPitchRoll {
                yaw: ChosenFixedPointType::from_num(0),
                pitch: ChosenFixedPointType::from_num(0),
                roll: ChosenFixedPointType::from_num(0),
                lift: ChosenFixedPointType::from_num(0),
                pressure: ChosenFixedPointType::from_num(0),
            },
        }
    }
    pub fn reset(&mut self) {
        *self = CalibrationState::new();
    }

    pub fn read_new_sample(&mut self) {
        let ypr_sample = YawPitchRoll::from(block!(read_dmp_bytes()).unwrap());

        let raw_read = read_raw().unwrap();

        self.accelerometer_sum = self.accelerometer_sum + raw_read.0;
        self.gyro_sum = self.gyro_sum + raw_read.1;

        self.ypr_sum = self.ypr_sum + ypr_sample;
        self.sample_cnt += I64F0::from_num(0);
    }

    pub fn finalize_calibration(&mut self) {
        self.accelerometer_offset = Axis {
            x: I32F0::from_num(self.accelerometer_sum.x / self.sample_cnt),
            y: I32F0::from_num(self.accelerometer_sum.y / self.sample_cnt),
            z: I32F0::from_num(self.accelerometer_sum.z / self.sample_cnt),
        };
        self.accelerometer_offset.z -= I32F0::from_num(LSB_FOR_ACCEL);

        self.gyro_offset = Axis {
            x: I32F0::from_num(self.gyro_sum.x / self.sample_cnt),
            y: I32F0::from_num(self.gyro_sum.y / self.sample_cnt),
            z: I32F0::from_num(self.gyro_sum.z / self.sample_cnt),
        };

        self.ypr_offset = YawPitchRoll {
            lift: ChosenFixedPointType::from_num(0),
            yaw: self.ypr_sum.yaw / ChosenFixedPointType::from_num(self.sample_cnt),
            pitch: self.ypr_sum.pitch / ChosenFixedPointType::from_num(self.sample_cnt),
            roll: self.ypr_sum.roll / ChosenFixedPointType::from_num(self.sample_cnt),
            pressure: ChosenFixedPointType::from_num(0),
        };
    }
    pub fn should_finish(&mut self) -> bool {
        return Instant::now().duration_since(self.start_time) >= CALIBRATION_TIME;
    }
}
