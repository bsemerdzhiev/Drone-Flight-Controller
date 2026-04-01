use core::ops::{Add, Div, Mul, Sub};

use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::mpu::structs::Quaternion;

use crate::util::{
    approx_funcs::{approx_atan2, approx_sqrt},
    constants_file::{
        DegreeType, PIDValuesType, QuaternionValuesType, SensorFixedType, TimeDifferenceType,
        MAX_LIFT, PITCH_DEGREE, RAD_TO_DEGREE, ROLL_DEGREE, YAW_RATE,
    },
    yaw_pitch_roll,
};

/// This struct holds the yaw, pitch, and roll that the drone things it is in.
/// The struct is currently implemented using `f32`, you may want to change this to use fixed point arithmetic.
#[derive(Debug, Copy, Clone)]
pub struct YawPitchRoll {
    pub lift: DegreeType,
    pub yaw: DegreeType,
    pub pitch: DegreeType,
    pub roll: DegreeType,

    pub pressure: SensorFixedType,
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

impl Mul<PIDValuesType> for YawPitchRoll {
    type Output = Self;

    fn mul(self, scalar: PIDValuesType) -> Self::Output {
        Self {
            lift: self.lift * DegreeType::from_num(scalar),
            yaw: self.yaw * DegreeType::from_num(scalar),
            pitch: self.pitch * DegreeType::from_num(scalar),
            roll: self.roll * DegreeType::from_num(scalar),
            pressure: self.pressure * SensorFixedType::from_num(scalar),
        }
    }
}

impl Div<PIDValuesType> for YawPitchRoll {
    type Output = Self;

    fn div(self, scalar: PIDValuesType) -> Self::Output {
        Self {
            lift: self.lift / DegreeType::from_num(scalar),
            yaw: self.yaw / DegreeType::from_num(scalar),
            pitch: self.pitch / DegreeType::from_num(scalar),
            roll: self.roll / DegreeType::from_num(scalar),
            pressure: self.pressure / SensorFixedType::from_num(scalar),
        }
    }
}

impl Mul<[PIDValuesType; 4]> for YawPitchRoll {
    type Output = Self;

    fn mul(self, scalar: [PIDValuesType; 4]) -> Self::Output {
        Self {
            lift: self.lift,
            yaw: self.yaw * DegreeType::from_num(scalar[0]),
            pitch: self.pitch * DegreeType::from_num(scalar[1]),
            roll: self.roll * DegreeType::from_num(scalar[2]),
            pressure: self.pressure * SensorFixedType::from_num(scalar[3]),
        }
    }
}

impl From<Quaternion> for YawPitchRoll {
    /// Creates a YawPitchRoll from a Quaternion
    fn from(q: Quaternion) -> Self {
        let Quaternion { w, x, y, z } = q;
        let w = QuaternionValuesType::from_num(w);
        let x = QuaternionValuesType::from_num(x);
        let y = QuaternionValuesType::from_num(y);
        let z = QuaternionValuesType::from_num(z);

        let gx: QuaternionValuesType = 2 * (x * z - w * y);
        let gy: QuaternionValuesType = 2 * (w * x + y * z);
        let gz: QuaternionValuesType = w * w - x * x - y * y + z * z;

        let yaw: QuaternionValuesType = approx_atan2(
            QuaternionValuesType::from_num(2) * (w * z + x * y),
            QuaternionValuesType::from_num(1) - QuaternionValuesType::from_num(2) * (y * y + z * z),
        ) / 2;
        // let yaw =
        // micromath::F32Ext::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) / 2.0;

        // pitch: (nose up/down, about Y axis)
        // let pitch = micromath::F32Ext::atan2(gx, micromath::F32Ext::sqrt(gy * gy + gz * gz));
        let pitch: QuaternionValuesType = approx_atan2(gx, approx_sqrt(gy * gy + gz * gz));

        // roll: (tilt left/right, about X axis)
        // let roll = micromath::F32Ext::atan2(gy, gz);
        let roll: QuaternionValuesType = approx_atan2(gy, gz);

        Self {
            lift: DegreeType::from_num(0),
            yaw: DegreeType::from_num(yaw),
            pitch: DegreeType::from_num(pitch),
            roll: DegreeType::from_num(roll),
            pressure: SensorFixedType::from_num(0),
        }
    }
}

impl YawPitchRoll {
    pub fn new() -> Self {
        YawPitchRoll {
            lift: DegreeType::from_num(0),
            yaw: DegreeType::from_num(0),
            pitch: DegreeType::from_num(0),
            roll: DegreeType::from_num(0),
            pressure: SensorFixedType::from_num(0),
        }
    }
    pub fn from_manual_input(input: &ManualInput) -> Self {
        Self {
            lift: MAX_LIFT * DegreeType::from_num(input.get_lift()),
            yaw: YAW_RATE * DegreeType::from_num(input.get_yaw()),
            pitch: PITCH_DEGREE * DegreeType::from_num(input.get_pitch()),
            roll: ROLL_DEGREE * DegreeType::from_num(input.get_roll()),
            pressure: SensorFixedType::from_num(0),
        }
    }
    pub fn calculate_rate_per_sec(
        &self,
        prev_sample: YawPitchRoll,
        duration_in_sec: TimeDifferenceType,
    ) -> Self {
        // let PI: f32 = micromath::F32Ext::acos(-1.0);
        // let TO_DEGREES: f32 = 180.0 / PI;
        YawPitchRoll {
            lift: DegreeType::from_num(0),
            yaw: (RAD_TO_DEGREE * (self.yaw - prev_sample.yaw))
                / DegreeType::from_num(duration_in_sec),
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
