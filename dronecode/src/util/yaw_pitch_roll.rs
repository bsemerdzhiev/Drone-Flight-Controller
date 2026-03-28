use core::ops::{Add, Div, Mul, Sub};

use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::mpu::structs::Quaternion;

/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub lift: f32,
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,

    pub pressure: f32,
}

impl Sub for YawPitchRoll {
    type Output = Self;

    fn sub(self, other: YawPitchRoll) -> Self::Output {
        Self {
            lift: self.lift - other.lift,
            yaw: self.yaw - other.yaw,
            pitch: self.pitch - other.pitch,
            roll: self.roll - other.roll,
            pressure: self.pressure - other.pressure,
        }
    }
}

impl Add for YawPitchRoll {
    type Output = Self;

    fn add(self, other: YawPitchRoll) -> Self::Output {
        Self {
            lift: self.lift + other.lift,
            yaw: self.yaw + other.yaw,
            pitch: self.pitch + other.pitch,
            roll: self.roll + other.roll,
            pressure: self.pressure + other.pressure,
        }
    }
}

impl Mul<f32> for YawPitchRoll {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self::Output {
        Self {
            lift: self.lift * scalar,
            yaw: self.yaw * scalar,
            pitch: self.pitch * scalar,
            roll: self.roll * scalar,
            pressure: self.pressure * scalar,
        }
    }
}

impl Div<f32> for YawPitchRoll {
    type Output = Self;

    fn div(self, scalar: f32) -> Self::Output {
        Self {
            lift: self.lift / scalar,
            yaw: self.yaw / scalar,
            pitch: self.pitch / scalar,
            roll: self.roll / scalar,
            pressure: self.pressure / scalar,
        }
    }
}

impl Mul<[f32; 4]> for YawPitchRoll {
    type Output = Self;

    fn mul(self, scalar: [f32; 4]) -> Self::Output {
        Self {
            lift: self.lift,
            yaw: self.yaw * scalar[0],
            pitch: self.pitch * scalar[1],
            roll: self.roll * scalar[2],
            pressure: self.pressure * scalar[3],
        }
    }
}

impl From<Quaternion> for YawPitchRoll {
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;
        let w: f32 = w.to_num();
        let x: f32 = x.to_num();
        let y: f32 = y.to_num();
        let z: f32 = z.to_num();

        let gx = 2.0 * (x * z - w * y);
        let gy = 2.0 * (w * x + y * z);
        let gz = w * w - x * x - y * y + z * z;

        // yaw: (about Z axis)
        let yaw =
            micromath::F32Ext::atan2(2.0 * x * y - 2.0 * w * z, 2.0 * w * w + 2.0 * x * x - 1.0);

        // pitch: (nose up/down, about Y axis)
        let pitch = micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        let roll = micromath::F32Ext::atan2(gy, gz);

        Self {
            lift: 0f32,
            yaw,
            pitch,
            roll,
            pressure: 0f32,
        }
    }
}

impl YawPitchRoll {
    pub fn new() -> Self {
        YawPitchRoll {
            lift: 0f32,
            yaw: 0f32,
            pitch: 0f32,
            roll: 0f32,
            pressure: 0f32,
        }
    }
    pub fn from_manual_input(input: &ManualInput) -> Self {
        Self {
            lift: input.get_lift() as f32,
            yaw: input.get_yaw() as f32,
            pitch: input.get_pitch() as f32,
            roll: input.get_roll() as f32,
            pressure: 0f32,
        }
    }
    pub fn calculate_rate_per_sec(&self, prev_sample: YawPitchRoll, duration_in_sec: f32) -> Self {
        YawPitchRoll {
            lift: 0f32,
            yaw: (self.yaw - prev_sample.yaw) / duration_in_sec,
            pitch: (self.pitch - prev_sample.pitch),
            roll: (self.roll - prev_sample.roll),
            pressure: self.pressure,
        }
    }
}
