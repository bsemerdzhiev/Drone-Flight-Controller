use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use fixed::types::{I26F6, I2F30};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;

use tudelft_quadrupel::{
    mpu::{
        read_dmp_bytes,
        structs::{Accel, Gyro},
    },
    time::Instant,
};

const RAD_TO_DEGREE: I26F6 = I26F6::lit("57.2957795");

/*
* Reads IMU data from the DMP.
* Data is read in Quaternions
* and then parsed into YawPitchRoll(euler angles)
*
* Since we are interested in the rate of change,
* store the previous reading along with the time
* it was taken to compute the rate of change per a second.
*/
pub struct DmpReadings {
    last_sampled_time: Option<Instant>,
    last_sample: Option<YawPitchRoll<I2F30, I2F30>>,

    calibration_offset: YawPitchRoll<I2F30, I2F30>,
}

impl DmpReadings {
    pub fn new(offset: YawPitchRoll<I2F30, I2F30>) -> Self {
        DmpReadings {
            last_sampled_time: None,
            last_sample: None,
            calibration_offset: offset,
        }
    }
}

impl ImuHandler for DmpReadings {
    fn get_reading(&mut self) -> YawPitchRoll<I26F6, I2F30> {
        let sampled_dmp_res = block!(read_dmp_bytes());

        let sampled_quaternion = sampled_dmp_res.unwrap();

        let mut sampled_yaw_pitch_roll = YawPitchRoll::<I2F30, I2F30>::from(sampled_quaternion);

        sampled_yaw_pitch_roll = sampled_yaw_pitch_roll - self.calibration_offset;

        if self.last_sampled_time.is_none() {
            self.last_sampled_time = Some(Instant::now());
            self.last_sample = Some(sampled_yaw_pitch_roll);

            return YawPitchRoll::<I26F6, I2F30>::new();
        }
        let current_time: Instant = Instant::now();

        let passed_time = I2F30::from_num(
            current_time
                .duration_since(self.last_sampled_time.unwrap())
                .as_secs_f32(),
        );

        // derive rate
        let calculated_rate = sampled_yaw_pitch_roll.calculate_rate_per_sec::<I26F6, I2F30>(
            self.last_sample.unwrap(),
            passed_time,
            RAD_TO_DEGREE,
        );

        self.last_sampled_time = Some(current_time);
        self.last_sample = Some(sampled_yaw_pitch_roll);

        return calculated_rate;
    }
    fn append_new_reading(&mut self) {}
}
