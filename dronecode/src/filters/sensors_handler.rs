use crate::util::yaw_pitch_roll::YawPitchRoll;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::structs::Gyro;

pub trait ImuHandler {
    fn get_reading(&mut self) -> Option<YawPitchRoll>;
    fn append_new_reading(&mut self);
}
