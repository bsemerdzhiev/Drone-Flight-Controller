use cordic::CordicNumber;
use fixed::{
    traits::{Fixed, FixedSigned},
    types::{I16F16, I32F0},
};
use my_hdlc::pc_command::{ManualDroneInput, ManualDroneTrimsEnums};
use tudelft_quadrupel::{led::Led::Yellow, time::Instant};

use crate::util::yaw_pitch_roll::YawPitchRoll;

pub type ControllerValues = I16F16;

const DEGREE_TO_RAD: ControllerValues = ControllerValues::lit("0.0174");

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
const HOVER_FORCE: ControllerValues = ControllerValues::lit("7.5");
const DRONE_WEIGHT: f32 = 2f32;

pub struct PIDController<T, Y>
where
    T: FixedSigned + CordicNumber,
    Y: FixedSigned,
{
    pub k_p: [ControllerValues; 4],
    pub k_i: [ControllerValues; 4],
    pub k_d: [ControllerValues; 4],

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
            k_p: [
                ControllerValues::ZERO,
                ControllerValues::ZERO,
                ControllerValues::ZERO,
                ControllerValues::ZERO,
            ],
            k_i: [
                ControllerValues::ZERO,
                ControllerValues::ZERO,
                ControllerValues::ZERO,
                ControllerValues::ZERO,
            ],

            k_d: [
                ControllerValues::ZERO,
                ControllerValues::ZERO,
                ControllerValues::ZERO,
                ControllerValues::ZERO,
            ],

            prev_error: YawPitchRoll::<T, Y>::new(),
            integration_build_up: YawPitchRoll::<T, Y>::new(),

            last_timestamp: Instant::now(),
        }
    }

    pub fn compute_pid_correction(
        &mut self,
        input: YawPitchRoll<T, Y>,
        target: YawPitchRoll<T, Y>,
        controller_flags: u8,
    ) -> YawPitchRoll<T, Y>
    where
        T: Fixed + CordicNumber,
        Y: Fixed,
    {
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
            result = result + (calculated_error.mul_pid_values::<ControllerValues>(self.k_p));
        }

        // compute D part
        if ((controller_flags & (ControllerFlags::AddD as u8)) != 0) {
            result = result
                + (((calculated_error - self.prev_error) / delta_t)
                    .mul_pid_values::<ControllerValues>(self.k_d));
            self.prev_error = calculated_error;
        }

        // compute I part
        if ((controller_flags & (ControllerFlags::AddI as u8)) != 0) {
            self.integration_build_up = self.integration_build_up + (calculated_error * delta_t);
            result = result
                + (self
                    .integration_build_up
                    .mul_pid_values::<ControllerValues>(self.k_i));
        }
        // update the timestamp
        self.last_timestamp = current_time;

        // unit of result.pressure in the end is m/s^2(in other words acceleration)
        // units of result.lift become Newtons

        // calculate lift based on pressure calculations
        let tilt_compensation: T = cordic::cos(input.pitch * T::from_num(DEGREE_TO_RAD))
            * cordic::cos(input.roll * T::from_num(DEGREE_TO_RAD));
        result.lift = (T::from_num(HOVER_FORCE) + T::from_num(result.pressure)) / tilt_compensation;

        return result;
    }

    pub fn reset_error(&mut self) {
        self.integration_build_up = YawPitchRoll::<T, Y>::new();
        self.prev_error = YawPitchRoll::<T, Y>::new();
    }
}
