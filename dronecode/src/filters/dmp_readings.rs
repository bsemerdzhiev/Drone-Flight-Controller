use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::util::axis::Axis;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use cordic::CordicNumber;
use fixed::traits::{Fixed, FixedSigned};
use fixed::types::{I16F16, I26F6, I2F30, I32F0, I4F28, I8F24};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;

use tudelft_quadrupel::mpu::read_raw;
use tudelft_quadrupel::{
    mpu::{
        read_dmp_bytes,
        structs::{Accel, Gyro},
    },
    time::Instant,
};

const RAD_TO_DEGREE: I8F24 = I8F24::lit("57.2957795");
const LSB_FOR_GYRO: I16F16 = I16F16::lit("16.4");

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
    pub calibration_offset: YawPitchRoll<I8F24, I8F24>,
    pub calibration_offset_raw_read: Axis<I16F16>,
}

impl DmpReadings {
    pub fn new(offset: YawPitchRoll<I8F24, I8F24>, offset_raw: Axis<I16F16>) -> Self {
        DmpReadings {
            calibration_offset: offset,
            calibration_offset_raw_read: offset_raw,
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

        // ----------------------------------
        // for the yaw component
        let mut raw_read = read_raw().unwrap();

        let yaw: T = (T::from_num(raw_read.1.z) - T::from_num(self.calibration_offset_raw_read.z))
            / T::from_num(LSB_FOR_GYRO);

        // ----------------------------------

        let sampled_quaternion = sampled_dmp_res.unwrap();

        let mut sampled_yaw_pitch_roll =
            YawPitchRoll::<I8F24, I8F24>::from(sampled_quaternion) * RAD_TO_DEGREE;

        sampled_yaw_pitch_roll = sampled_yaw_pitch_roll - self.calibration_offset;

        // derive rate
        return YawPitchRoll::<T, Y> {
            lift: T::from_num(0),
            yaw: T::from_num(yaw),
            pitch: T::from_num(sampled_yaw_pitch_roll.pitch),
            roll: T::from_num(sampled_yaw_pitch_roll.roll),
            pressure: Y::from_num(0),
        };
    }
    fn append_new_reading(&mut self) {}
}
