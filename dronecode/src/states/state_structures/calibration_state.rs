use core::{ops::Add, time::Duration};

use tudelft_quadrupel::{
    barometer::read_pressure,
    block,
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
pub struct CalibrationState {
    accelerometer_sum: Axis<i64>,
    gyro_sum: Axis<i64>,

    ypr_sum: YawPitchRoll,
    sample_cnt: i32,

    pub start_time: Instant,

    pub accelerometer_offset: Axis<i32>,
    pub gyro_offset: Axis<i32>,
    pub ypr_offset: YawPitchRoll,
}

impl CalibrationState {
    pub fn new() -> Self {
        Self {
            accelerometer_sum: Axis::<i64>::default(),
            gyro_sum: Axis::<i64>::default(),
            ypr_sum: YawPitchRoll::new(),
            sample_cnt: 0,
            start_time: Instant::now(),

            accelerometer_offset: Axis { x: 0, y: 0, z: 0 },
            gyro_offset: Axis { x: 0, y: 0, z: 0 },

            ypr_offset: YawPitchRoll {
                yaw: 0.0,
                pitch: 0.0,
                roll: 0.0,
                lift: 0.0,
                pressure: 0.0,
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
        self.sample_cnt += 1;
    }

    pub fn finalize_calibration(&mut self) {
        self.accelerometer_offset = Axis {
            x: (self.accelerometer_sum.x / self.sample_cnt as i64) as i32,
            y: (self.accelerometer_sum.y / self.sample_cnt as i64) as i32,
            z: (self.accelerometer_sum.z / self.sample_cnt as i64) as i32,
        };
        self.accelerometer_offset.z -= LSB_FOR_ACCEL;

        self.gyro_offset = Axis {
            x: (self.gyro_sum.x / self.sample_cnt as i64) as i32,
            y: (self.gyro_sum.y / self.sample_cnt as i64) as i32,
            z: (self.gyro_sum.z / self.sample_cnt as i64) as i32,
        };

        self.ypr_offset = YawPitchRoll {
            lift: 0.0,
            yaw: self.ypr_sum.yaw / self.sample_cnt as f32,
            pitch: self.ypr_sum.pitch / self.sample_cnt as f32,
            roll: self.ypr_sum.roll / self.sample_cnt as f32,
            pressure: 0.0,
        };
    }
    pub fn should_finish(&mut self) -> bool {
        return Instant::now().duration_since(self.start_time) >= CALIBRATION_TIME;
    }
}
