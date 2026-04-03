use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use cordic::CordicNumber;
use fixed::traits::{Fixed, FixedSigned};
use fixed::types::{I16F16, I26F6, I2F30, I32F0, I4F28, I8F24};
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
    last_sample: Option<YawPitchRoll<I8F24, I8F24>>,

    calibration_offset: YawPitchRoll<I8F24, I8F24>,
}

impl DmpReadings {
    pub fn new(offset: YawPitchRoll<I8F24, I8F24>) -> Self {
        DmpReadings {
            last_sampled_time: None,
            last_sample: None,
            calibration_offset: offset,
        }
    }
}

impl ImuHandler for DmpReadings {
    fn get_reading<T, Y>(&mut self) -> YawPitchRoll<T, Y>
    where
        T: FixedSigned + CordicNumber,
        Y: FixedSigned,
    {
        let sampled_dmp_res = block!(read_dmp_bytes());

        let sampled_quaternion = sampled_dmp_res.unwrap();

        let mut sampled_yaw_pitch_roll = YawPitchRoll::<I8F24, I8F24>::from(sampled_quaternion);

        sampled_yaw_pitch_roll = sampled_yaw_pitch_roll - self.calibration_offset;

        if self.last_sampled_time.is_none() {
            self.last_sampled_time = Some(Instant::now());
            self.last_sample = Some(sampled_yaw_pitch_roll);

            return YawPitchRoll::<T, Y>::new();
        }
        let current_time: Instant = Instant::now();

        let passed_time: I8F24 = I8F24::from_num(
            (I16F16::from_num(
                current_time
                    .duration_since(self.last_sampled_time.unwrap())
                    .as_micros() as u32,
            ) / I16F16::from_num(1000))
                / I16F16::from_num(1000),
        );

        // derive rate
        let calculated_rate = sampled_yaw_pitch_roll.calculate_rate_per_sec::<T, Y>(
            self.last_sample.unwrap(),
            passed_time,
            T::from_num(RAD_TO_DEGREE),
        );

        self.last_sampled_time = Some(current_time);
        self.last_sample = Some(sampled_yaw_pitch_roll);

        return calculated_rate;
    }
    fn append_new_reading(&mut self) {}
}
