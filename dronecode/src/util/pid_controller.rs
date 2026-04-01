use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::time::Instant;

use crate::util::{constants_file::ChosenFixedPointType, yaw_pitch_roll::YawPitchRoll};

pub const K_P: [ChosenFixedPointType; 4] = [
    ChosenFixedPointType::lit("4"),
    ChosenFixedPointType::lit("0.005"),
    ChosenFixedPointType::lit("0.005"),
    ChosenFixedPointType::lit("8"),
];
pub const K_I: [ChosenFixedPointType; 4] = [
    ChosenFixedPointType::lit("0"),
    ChosenFixedPointType::lit("0"),
    ChosenFixedPointType::lit("0"),
    ChosenFixedPointType::lit("0"),
];
pub const K_D: [ChosenFixedPointType; 4] = [
    ChosenFixedPointType::lit("0"),
    ChosenFixedPointType::lit("0"),
    ChosenFixedPointType::lit("0"),
    ChosenFixedPointType::lit("0"),
];

pub fn add_trims(
    manual_input: &ManualInput,
) -> (
    [ChosenFixedPointType; 4],
    [ChosenFixedPointType; 4],
    [ChosenFixedPointType; 4],
) {
    let mut k_p: [ChosenFixedPointType; 4] = K_P;
    let mut k_i: [ChosenFixedPointType; 4] = K_I;
    let mut k_d: [ChosenFixedPointType; 4] = K_D;

    k_p[0] += ChosenFixedPointType::from_num(manual_input.yaw_p_trim);

    k_p[1] += ChosenFixedPointType::from_num(manual_input.roll_pitch_p_trim);
    k_p[2] += ChosenFixedPointType::from_num(manual_input.roll_pitch_p_trim);

    k_d[1] += ChosenFixedPointType::from_num(manual_input.roll_pitch_d_trim);
    k_d[2] += ChosenFixedPointType::from_num(manual_input.roll_pitch_d_trim);

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
const DRONE_WEIGHT: ChosenFixedPointType = ChosenFixedPointType::lit("0.5");

const GRAVITY_CONSTANT: ChosenFixedPointType = ChosenFixedPointType::lit("9.8");

pub struct PIDController {
    prev_error: YawPitchRoll,
    integration_build_up: YawPitchRoll,

    last_timestamp: Instant,
}

impl PIDController {
    pub fn new() -> Self {
        PIDController {
            prev_error: YawPitchRoll::new(),
            integration_build_up: YawPitchRoll::new(),

            last_timestamp: Instant::now(),
        }
    }

    pub fn compute_pid_correction(
        &mut self,
        input: YawPitchRoll,
        target: YawPitchRoll,
        k_p: [ChosenFixedPointType; 4],
        k_i: [ChosenFixedPointType; 4],
        k_d: [ChosenFixedPointType; 4],
        controller_flags: u8,
    ) -> YawPitchRoll {
        /*
         *  for calculations, check
         *  https://harikrishnansuresh.github.io/assets/QuadcopterControlFinalVersion.pdf
         */

        let mut result = YawPitchRoll::new();
        let calculated_error = (target - input);

        let current_time = Instant::now();
        let delta_t = ChosenFixedPointType::from_num(
            current_time
                .duration_since(self.last_timestamp)
                .as_secs_f32()
                .clamp(0.001, 0.02),
        );

        // compute P part
        if ((controller_flags & (ControllerFlags::AddP as u8)) != 0) {
            result = result + (calculated_error * k_p);
        }

        // compute D part
        if ((controller_flags & (ControllerFlags::AddD as u8)) != 0) {
            result = result + (((calculated_error - self.prev_error) / delta_t) * k_d);
            self.prev_error = calculated_error;
        }

        // compute I part
        if ((controller_flags & (ControllerFlags::AddI as u8)) != 0) {
            self.integration_build_up = self.integration_build_up + (calculated_error * delta_t);
            result = result + (self.integration_build_up * k_i);
        }
        // update the timestamp
        self.last_timestamp = current_time;

        // unit of result.pressure in the end is m/s^2(in other words acceleration)
        // units of result.lift become Newtons

        /* calculate lift based on pressure calculations
         *
         *
         *                          since we want to hover
         *                                   |
         *            measured drone         |             calculated
         *               weight              v               correction
         */
        result.lift = DRONE_WEIGHT * GRAVITY_CONSTANT + (result.pressure);

        return result;
    }
}
