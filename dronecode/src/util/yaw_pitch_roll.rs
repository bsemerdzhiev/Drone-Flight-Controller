use core::ops::{Add, Div, Mul, Sub};

use fixed::traits::{Fixed, FixedSigned};
use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::mpu::structs::Quaternion;

use crate::util::{
    approx_funcs::{approx_sqrt, atan2_cordic},
    constants_file::{MAX_LIFT, PITCH_DEGREE, ROLL_DEGREE, YAW_RATE},
};

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
    T: FixedSigned,
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

        let yaw: T = atan2_cordic::<T>(
            T::from_num(2) * (w * z + x * y),
            T::from_num(1) - T::from_num(2) * (y * y + z * z),
        ) / T::from_num(2);
        // let yaw = micromath::F32Ext::atan2(
        // 2.0 * (w.to_num::<f32>() * z.to_num::<f32>() + x.to_num::<f32>() * y.to_num::<f32>()),
        // 1.0 - 2.0
        // * (y.to_num::<f32>() * y.to_num::<f32>() + z.to_num::<f32>() * z.to_num::<f32>()),
        // ) / 2.0;

        // pitch: (nose up/down, about Y axis)
        // let pitch = micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));
        let pitch: T = atan2_cordic(gx, approx_sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        // let roll = micromath::F32Ext::atan2(gy, gz);
        let roll: T = atan2_cordic(gy, gz);

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
    T: FixedSigned,
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
    pub fn from_manual_input(input: &ManualInput) -> Self {
        Self {
            lift: T::from_num(MAX_LIFT) * T::from_num(input.get_lift()),
            yaw: T::from_num(YAW_RATE) * T::from_num(input.get_yaw()),
            pitch: T::from_num(PITCH_DEGREE) * T::from_num(input.get_pitch()),
            roll: T::from_num(ROLL_DEGREE) * T::from_num(input.get_roll()),
            pressure: Y::from_num(0),
        }
    }
    pub fn calculate_rate_per_sec<W_T, W_Y>(
        &self,
        prev_sample: YawPitchRoll<T, Y>,
        duration_in_sec: T,
        rad_to_degree: W_T,
    ) -> YawPitchRoll<W_T, W_Y>
    where
        T: FixedSigned,
        Y: FixedSigned,
        W_T: FixedSigned,
        W_Y: FixedSigned,
    {
        YawPitchRoll::<W_T, W_Y> {
            lift: W_T::from_num(0),
            yaw: (rad_to_degree * W_T::from_num((self.yaw - prev_sample.yaw) / duration_in_sec)),
            pitch: (rad_to_degree * W_T::from_num(self.pitch)),
            roll: (rad_to_degree * W_T::from_num(self.roll)),
            pressure: W_Y::from_num(0),
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
