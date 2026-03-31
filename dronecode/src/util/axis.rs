use core::ops::Add;

use tudelft_quadrupel::mpu::structs::{Accel, Gyro};

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Axis<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl Add<Accel> for Axis<i64> {
    type Output = Axis<i64>;

    fn add(self, input: Accel) -> Self::Output {
        Axis::<i64> {
            x: self.x + input.x as i64,
            y: self.y + input.y as i64,
            z: self.z + input.z as i64,
        }
    }
}

impl Add<Gyro> for Axis<i64> {
    type Output = Axis<i64>;

    fn add(self, input: Gyro) -> Self::Output {
        Axis::<i64> {
            x: self.x + input.x as i64,
            y: self.y + input.y as i64,
            z: self.z + input.z as i64,
        }
    }
}

impl From<Accel> for Axis<i32> {
    fn from(input: Accel) -> Axis<i32> {
        Axis {
            x: input.x as i32,
            y: input.y as i32,
            z: input.z as i32,
        }
    }
}

impl From<Gyro> for Axis<i32> {
    fn from(input: Gyro) -> Axis<i32> {
        Axis {
            x: input.x as i32,
            y: input.y as i32,
            z: input.z as i32,
        }
    }
}

impl Axis<i32> {
    pub fn to_array(&mut self) -> [i32; 3] {
        return [self.x, self.y, self.z];
    }
}
