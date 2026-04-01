use core::ops::{Add, Div, Mul, Sub};

use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::mpu::structs::Quaternion;

use crate::util::{
    approx_funcs::{approx_atan2, approx_sqrt},
    constants_file::{
        ChosenFixedPointType, MAX_LIFT, PITCH_DEGREE, RAD_TO_DEGREE, ROLL_DEGREE, YAW_RATE,
    },
    yaw_pitch_roll,
};

/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub lift: ChosenFixedPointType,
    pub yaw: ChosenFixedPointType,
    pub pitch: ChosenFixedPointType,
    pub roll: ChosenFixedPointType,

    pub pressure: ChosenFixedPointType,
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

impl Mul<ChosenFixedPointType> for YawPitchRoll {
    type Output = Self;

    fn mul(self, scalar: ChosenFixedPointType) -> Self::Output {
        Self {
            lift: self.lift * scalar,
            yaw: self.yaw * scalar,
            pitch: self.pitch * scalar,
            roll: self.roll * scalar,
            pressure: self.pressure * scalar,
        }
    }
}

impl Div<ChosenFixedPointType> for YawPitchRoll {
    type Output = Self;

    fn div(self, scalar: ChosenFixedPointType) -> Self::Output {
        Self {
            lift: self.lift / scalar,
            yaw: self.yaw / scalar,
            pitch: self.pitch / scalar,
            roll: self.roll / scalar,
            pressure: self.pressure / scalar,
        }
    }
}

impl Mul<[ChosenFixedPointType; 4]> for YawPitchRoll {
    type Output = Self;

    fn mul(self, scalar: [ChosenFixedPointType; 4]) -> Self::Output {
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
        let w = ChosenFixedPointType::from_num(w);
        let x = ChosenFixedPointType::from_num(x);
        let y = ChosenFixedPointType::from_num(y);
        let z = ChosenFixedPointType::from_num(z);

        let gx: ChosenFixedPointType = 2 * (x * z - w * y);
        let gy: ChosenFixedPointType = 2 * (w * x + y * z);
        let gz: ChosenFixedPointType = w * w - x * x - y * y + z * z;

        let yaw = approx_atan2(
            ChosenFixedPointType::from_num(2) * (w * z + x * y),
            ChosenFixedPointType::from_num(1) - ChosenFixedPointType::from_num(2) * (y * y + z * z),
        ) / 2;
        // let yaw =
        // micromath::F32Ext::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) / 2.0;

        // pitch: (nose up/down, about Y axis)
        // let pitch = micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));
        let pitch = approx_atan2(gx, approx_sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        // let roll = micromath::F32Ext::atan2(gy, gz);
        let roll = approx_atan2(gy, gz);

        Self {
            lift: ChosenFixedPointType::from_num(0),
            yaw,
            pitch,
            roll,
            pressure: ChosenFixedPointType::from_num(0),
        }
    }
}

impl YawPitchRoll {
    pub fn new() -> Self {
        YawPitchRoll {
            lift: ChosenFixedPointType::from_num(0),
            yaw: ChosenFixedPointType::from_num(0),
            pitch: ChosenFixedPointType::from_num(0),
            roll: ChosenFixedPointType::from_num(0),
            pressure: ChosenFixedPointType::from_num(0),
        }
    }
    pub fn from_manual_input(input: &ManualInput) -> Self {
        Self {
            lift: MAX_LIFT * ChosenFixedPointType::from_num(input.get_lift()),
            yaw: YAW_RATE * ChosenFixedPointType::from_num(input.get_yaw()),
            pitch: PITCH_DEGREE * ChosenFixedPointType::from_num(input.get_pitch()),
            roll: ROLL_DEGREE * ChosenFixedPointType::from_num(input.get_roll()),
            pressure: ChosenFixedPointType::from_num(0),
        }
    }
    pub fn calculate_rate_per_sec(
        &self,
        prev_sample: YawPitchRoll,
        duration_in_sec: ChosenFixedPointType,
    ) -> Self {
        // let PI: f32 = micromath::F32Ext::acos(-1.0);
        // let TO_DEGREES: f32 = 180.0 / PI;
        YawPitchRoll {
            lift: ChosenFixedPointType::from_num(0),
            yaw: (RAD_TO_DEGREE * (self.yaw - prev_sample.yaw)) / duration_in_sec,
            pitch: (RAD_TO_DEGREE * self.pitch),
            roll: (RAD_TO_DEGREE * self.roll),
            pressure: self.pressure,
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
