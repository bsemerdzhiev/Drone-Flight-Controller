use core::{ops::Add, time::Duration};

use tudelft_quadrupel::{
    mpu::structs::{Accel, Gyro},
    time::Instant,
};

use crate::util::yaw_pitch_roll::YawPitchRoll;

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Axis<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl Add<Accel> for Axis<i32> {
    type Output = Axis<i32>;

    fn add(self, input: Accel) -> Self::Output {
        Axis::<i32> {
            x: self.x + input.x as i32,
            y: self.y + input.y as i32,
            z: self.z + input.z as i32,
        }
    }
}

impl Add<Gyro> for Axis<i32> {
    type Output = Axis<i32>;

    fn add(self, input: Gyro) -> Self::Output {
        Axis::<i32> {
            x: self.x + input.x as i32,
            y: self.y + input.y as i32,
            z: self.z + input.z as i32,
        }
    }
}

const CALIBRATION_TIME: Duration = Duration::from_secs(5);

pub struct CalibrationState {
    accelerometer_sum: Axis<i32>,
    gyro_sum: Axis<i32>,
    ypr_sum: YawPitchRoll,
    sample_cnt: i32,
    pub start_time: Instant,
    pub accelerometer_offset: Accel,
    // pub gyro_offset: Gyro,
    pub ypr_offset: YawPitchRoll,
}

impl CalibrationState {
    pub fn new() -> Self {
        Self {
            accelerometer_sum: Axis::<i32>::default(),
            gyro_sum: Axis::<i32>::default(),
            ypr_sum: YawPitchRoll::new(),
            sample_cnt: 0,
            start_time: Instant::now(),
            accelerometer_offset: Accel { x: 0, y: 0, z: 0 },
            // gyro_offset: Gyro { x: 0, y: 0, z: 0 },
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

    pub fn read_new_sample(
        &mut self,
        accel_sample: Accel,
        gyro_sample: Gyro,
        ypr_sample: YawPitchRoll,
    ) {
        self.accelerometer_sum = self.accelerometer_sum + accel_sample;
        self.gyro_sum = self.gyro_sum + gyro_sample;
        self.ypr_sum = self.ypr_sum + ypr_sample;
        self.sample_cnt += 1;
    }

    pub fn finalize_calibration(&mut self) {
        self.accelerometer_offset = Accel {
            x: (self.accelerometer_sum.x / self.sample_cnt) as i16,
            y: (self.accelerometer_sum.y / self.sample_cnt) as i16,
            z: (self.accelerometer_sum.z / self.sample_cnt) as i16,
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
