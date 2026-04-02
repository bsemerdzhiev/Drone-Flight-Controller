use crate::util::yaw_pitch_roll::YawPitchRoll;
use fixed::traits::Fixed;
use fixed::traits::FixedSigned;
use fixed::types::I26F6;
use fixed::types::I4F28;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::structs::Gyro;

pub trait ImuHandler {
    fn get_reading<T, Y>(&mut self) -> YawPitchRoll<T, Y>
    where
        T: FixedSigned,
        Y: FixedSigned;
    fn append_new_reading(&mut self);
}
