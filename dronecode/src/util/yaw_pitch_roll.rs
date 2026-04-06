use core::{
    i16,
    ops::{Add, Div, Mul, Sub},
};

use cordic::{atan2, CordicNumber};
use fixed::traits::{Fixed, FixedSigned};
use my_hdlc::pc_command::ManualDroneInput;
use tudelft_quadrupel::mpu::structs::Quaternion;

use crate::util::{approx_funcs::approx_sqrt, MAX_LIFT, PITCH_DEGREE, ROLL_DEGREE, YAW_RATE};

// this structure is used to both store degrees and radians
// newton(m)
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
{
    pub lift: T,
    pub yaw: T,
    pub pitch: T,
    pub roll: T,

    pub pressure: Y,
}

impl<T, Y> Sub for YawPitchRoll<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
{
    type Output = Self;

    fn sub(self, other: YawPitchRoll<T, Y>) -> Self::Output {
        Self {
            lift: self.lift - other.lift,
            yaw: self.yaw - other.yaw,
            pitch: self.pitch - other.pitch,
            roll: self.roll - other.roll,
            pressure: self.pressure - other.pressure,
        }
    }
}

impl<T, Y> Add for YawPitchRoll<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
{
    type Output = Self;

    fn add(self, other: YawPitchRoll<T, Y>) -> Self::Output {
        Self {
            lift: self.lift + other.lift,
            yaw: self.yaw + other.yaw,
            pitch: self.pitch + other.pitch,
            roll: self.roll + other.roll,
            pressure: self.pressure + other.pressure,
        }
    }
}

impl<T, Y, Z> Mul<Z> for YawPitchRoll<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
    Z: FixedSigned,
{
    type Output = Self;

    fn mul(self, scalar: Z) -> Self::Output {
        Self {
            lift: self.lift * T::from_num(scalar),
            yaw: self.yaw * T::from_num(scalar),
            pitch: self.pitch * T::from_num(scalar),
            roll: self.roll * T::from_num(scalar),
            pressure: self.pressure * Y::from_num(scalar),
        }
    }
}

impl<T, Y, Z> Div<Z> for YawPitchRoll<T, Y>
where
    T: FixedSigned,
    Y: FixedSigned,
    Z: FixedSigned,
{
    type Output = Self;

    fn div(self, scalar: Z) -> Self::Output {
        Self {
            lift: self.lift / T::from_num(scalar),
            yaw: self.yaw / T::from_num(scalar),
            pitch: self.pitch / T::from_num(scalar),
            roll: self.roll / T::from_num(scalar),
            pressure: self.pressure / Y::from_num(scalar),
        }
    }
}

impl<T, Y> From<Quaternion> for YawPitchRoll<T, Y>
where
    T: FixedSigned + CordicNumber,
    Y: FixedSigned,
{
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;

        let w = T::from_num(w);
        let x = T::from_num(x);
        let y = T::from_num(y);
        let z = T::from_num(z);

        let gx: T = T::from_num(2) * (x * z - w * y);
        let gy: T = T::from_num(2) * (w * x + y * z);
        let gz: T = w * w - x * x - y * y + z * z;

        let yaw: T = atan2(
            T::from_num(2) * (w * z + x * y),
            T::from_num(1) - T::from_num(2) * (y * y + z * z),
        ) / T::from_num(2);

        // pitch: (nose up/down, about Y axis)
        // let pitch = micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));
        let pitch: T = atan2(gx, approx_sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        // let roll = micromath::F32Ext::atan2(gy, gz);
        let roll: T = atan2(gy, gz);

        Self {
            lift: T::from_num(0),
            yaw: T::from_num(yaw),
            pitch: T::from_num(pitch),
            roll: T::from_num(roll),
            pressure: Y::from_num(0),
        }
    }
}

impl<T, Y> YawPitchRoll<T, Y>
where
    T: FixedSigned + CordicNumber,
    Y: FixedSigned,
{
    pub fn new() -> Self {
        YawPitchRoll {
            lift: T::from_num(0),
            yaw: T::from_num(0),
            pitch: T::from_num(0),
            roll: T::from_num(0),
            pressure: Y::from_num(0),
        }
    }
    pub fn from_manual_input(input: &ManualDroneInput) -> Self {
        Self {
            lift: T::from_num(MAX_LIFT) * (T::from_num(input.lift) / T::from_num(i16::MAX)),
            yaw: T::from_num(YAW_RATE) * (T::from_num(input.yaw) / T::from_num(i16::MAX)),
            pitch: T::from_num(PITCH_DEGREE) * (T::from_num(input.pitch) / T::from_num(i16::MAX)),
            roll: T::from_num(ROLL_DEGREE) * (T::from_num(input.roll) / T::from_num(i16::MAX)),
            pressure: Y::from_num(0),
        }
    }
    pub fn mul_pid_values<Z>(&self, scalar: [Z; 4]) -> Self
    where
        Z: FixedSigned,
    {
        Self {
            lift: self.lift,
            yaw: self.yaw * T::from_num(scalar[0]),
            pitch: self.pitch * T::from_num(scalar[1]),
            roll: self.roll * T::from_num(scalar[2]),
            pressure: self.pressure * Y::from_num(scalar[3]),
        }
    }

    pub fn to_array(&mut self) -> [f32; 3] {
        return [
            self.yaw.to_num::<f32>(),
            self.pitch.to_num::<f32>(),
            self.roll.to_num::<f32>(),
        ];
    }
}
