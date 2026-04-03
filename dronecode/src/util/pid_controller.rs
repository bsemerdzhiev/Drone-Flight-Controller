use cordic::CordicNumber;
use fixed::{
    traits::{Fixed, FixedSigned},
    types::{I16F16, I32F0},
};
use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::time::Instant;

use crate::util::yaw_pitch_roll::YawPitchRoll;

type ControllerValues = I16F16;

pub const K_P: [ControllerValues; 4] = [
    ControllerValues::lit("1.5"),
    ControllerValues::lit("0.05"),
    ControllerValues::lit("0.05"),
    ControllerValues::lit("12"),
];
pub const K_I: [ControllerValues; 4] = [
    ControllerValues::lit("0"),
    ControllerValues::lit("0.0002"),
    ControllerValues::lit("0.0002"),
    ControllerValues::lit("0"),
];
pub const K_D: [ControllerValues; 4] = [
    ControllerValues::lit("0"),
    ControllerValues::lit("0.001"),
    ControllerValues::lit("0.001"),
    ControllerValues::lit("0"),
];

pub fn add_trims(
    manual_input: &ManualInput,
) -> (
    [ControllerValues; 4],
    [ControllerValues; 4],
    [ControllerValues; 4],
) {
    let mut k_p: [ControllerValues; 4] = K_P;
    let mut k_i: [ControllerValues; 4] = K_I;
    let mut k_d: [ControllerValues; 4] = K_D;

    k_p[0] += ControllerValues::from_num(manual_input.yaw_p_trim);

    k_p[1] += ControllerValues::from_num(manual_input.roll_pitch_p_trim);
    k_p[2] += ControllerValues::from_num(manual_input.roll_pitch_p_trim);

    k_d[1] += ControllerValues::from_num(manual_input.roll_pitch_d_trim);
    k_d[2] += ControllerValues::from_num(manual_input.roll_pitch_d_trim);

    return (k_p, k_i, k_d);
}

/*
* Selects the type of error correction
* we want to perform
*/
#[repr(u8)]
pub enum ControllerFlags {
    AddP = (1 << 0),
    AddD = (1 << 1),
    AddI = (1 << 2),
}

// in kg
// const DRONE_WEIGHT: ControllerValues = ControllerValues::lit("0.5");
// const GRAVITY_CONSTANT: ControllerValues = ControllerValues::lit("9.8");
const HOVER_FORCE: ControllerValues = ControllerValues::lit("18.5");
const DRONE_WEIGHT: f32 = 2f32;

pub struct PIDController<T, Y>
where
    T: FixedSigned + CordicNumber,
    Y: FixedSigned,
{
    prev_error: YawPitchRoll<T, Y>,
    integration_build_up: YawPitchRoll<T, Y>,

    last_timestamp: Instant,
}

impl<T, Y> PIDController<T, Y>
where
    T: FixedSigned + CordicNumber,
    Y: FixedSigned,
{
    pub fn new() -> Self {
        PIDController {
            prev_error: YawPitchRoll::<T, Y>::new(),
            integration_build_up: YawPitchRoll::<T, Y>::new(),

            last_timestamp: Instant::now(),
        }
    }

    pub fn compute_pid_correction(
        &mut self,
        input: YawPitchRoll<T, Y>,
        target: YawPitchRoll<T, Y>,
        k_p: [ControllerValues; 4],
        k_i: [ControllerValues; 4],
        k_d: [ControllerValues; 4],
        controller_flags: u8,
    ) -> YawPitchRoll<T, Y> {
        /*
         *  for calculations, check
         *  https://harikrishnansuresh.github.io/assets/QuadcopterControlFinalVersion.pdf
         */

        let mut result = YawPitchRoll::<T, Y>::new();
        let calculated_error = (target - input);

        let current_time = Instant::now();

        let delta_t: ControllerValues = ControllerValues::from_num(
            (I16F16::from_num(current_time.duration_since(self.last_timestamp).as_micros() as u32)
                / I16F16::from_num(1000))
                / I16F16::from_num(1000),
        );

        // compute P part
        if ((controller_flags & (ControllerFlags::AddP as u8)) != 0) {
            result = result + (calculated_error.mul_pid_values::<ControllerValues>(k_p));
        }

        // compute D part
        if ((controller_flags & (ControllerFlags::AddD as u8)) != 0) {
            result = result
                + (((calculated_error - self.prev_error) / delta_t)
                    .mul_pid_values::<ControllerValues>(k_d));
            self.prev_error = calculated_error;
        }

        // compute I part
        if ((controller_flags & (ControllerFlags::AddI as u8)) != 0) {
            self.integration_build_up = self.integration_build_up + (calculated_error * delta_t);
            result = result
                + (self
                    .integration_build_up
                    .mul_pid_values::<ControllerValues>(k_i));
        }
        // update the timestamp
        self.last_timestamp = current_time;

        // unit of result.pressure in the end is m/s^2(in other words acceleration)
        // units of result.lift become Newtons

        // calculate lift based on pressure calculations
        result.lift = T::from_num(HOVER_FORCE) + T::from_num(result.pressure);

        return result;
    }
}
