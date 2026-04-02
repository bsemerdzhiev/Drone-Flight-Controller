use core::ops::Add;

use fixed::types::I16F16;
use tudelft_quadrupel::{
    fixed::types::{I32F0, I64F0},
    mpu::structs::{Accel, Gyro},
};

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Axis<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl Add<Accel> for Axis<I32F0> {
    type Output = Axis<I32F0>;

    fn add(self, input: Accel) -> Self::Output {
        Axis::<I32F0> {
            x: self.x + I32F0::from_num(input.x),
            y: self.y + I32F0::from_num(input.y),
            z: self.z + I32F0::from_num(input.z),
        }
    }
}

impl Add<Gyro> for Axis<I32F0> {
    type Output = Axis<I32F0>;

    fn add(self, input: Gyro) -> Self::Output {
        Axis::<I32F0> {
            x: self.x + I32F0::from_num(input.x),
            y: self.y + I32F0::from_num(input.y),
            z: self.z + I32F0::from_num(input.z),
        }
    }
}

impl From<Accel> for Axis<I16F16> {
    fn from(input: Accel) -> Axis<I16F16> {
        Axis {
            x: I16F16::from_num(input.x),
            y: I16F16::from_num(input.y),
            z: I16F16::from_num(input.z),
        }
    }
}

impl From<Gyro> for Axis<I16F16> {
    fn from(input: Gyro) -> Axis<I16F16> {
        Axis {
            x: I16F16::from_num(input.x),
            y: I16F16::from_num(input.y),
            z: I16F16::from_num(input.z),
        }
    }
}

impl Axis<I16F16> {
    pub fn to_array(&mut self) -> [i32; 3] {
        return [
            self.x.to_num::<i32>(),
            self.y.to_num::<i32>(),
            self.z.to_num::<i32>(),
        ];
    }
}
