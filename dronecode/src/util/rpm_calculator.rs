use fixed::types::{I16F16, I26F6, I28F4, I29F3, I32F0, I32F32, I3F29, I40F24, I4F28};
use micromath::F32Ext;
use my_hdlc::{
    command::{DebugRpms, DeviceCommand},
    pc_command::ManualDroneInput,
};
use tudelft_quadrupel::{motor, uart::send_bytes};

use crate::util::{approx_funcs::approx_sqrt, yaw_pitch_roll::YawPitchRoll};

//------------------------------------------------------

type OmegaType = I28F4;

// chosen by trial and error in Desmos

const MIN_PWM: u16 = 200;

// pub const THRESHOLD_LIFT: f32 = 0.05;

pub const ThresholdLift: I16F16 = I16F16::lit("1.25");

fn map_rpm_square_to_pwm(lift_raw_value: I16F16, rpms_square: &mut [OmegaType]) {
    let MAX_RPMS: I16F16 = I16F16::from_num(980);

    let cur_maxes = I16F16::from_num(motor::get_motor_max());

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k: usize = 0;
    for x in rpms_square {
        let squared_number: I16F16 = I16F16::from_num(approx_sqrt(*x));

        let rpm_ratio = squared_number / MAX_RPMS;

        pwm_to_set[k] = (cur_maxes * rpm_ratio).to_num::<u16>();

        k += 1;
    }

    //if lift is below threshold, then all motors are off
    if lift_raw_value < ThresholdLift {
        for cur_motor_rpm in &mut pwm_to_set {
            *cur_motor_rpm = 0;
        }
    } else {
        for cur_motor_rpm in &mut pwm_to_set {
            *cur_motor_rpm = MIN_PWM.max(*cur_motor_rpm);
        }
    }

    motor::set_motors(pwm_to_set);
}

const THR_DIV: OmegaType = OmegaType::lit("1250");
const DRG_DIV: OmegaType = OmegaType::lit("17857.286");

pub fn actuate_motors_with_direct_joystick_input(
    input_from_controller: &YawPitchRoll<I16F16, I16F16>,
    raw_lift: I16F16,
) {
    let N = OmegaType::from_num(input_from_controller.yaw);
    let M = OmegaType::from_num(input_from_controller.pitch);
    let Z = OmegaType::from_num(-input_from_controller.lift);
    let L = OmegaType::from_num(input_from_controller.roll);

    let omega_one: OmegaType = Z + M - N;
    let omega_two: OmegaType = Z + L + N;
    let omega_three: OmegaType = Z - M - N;
    let omega_four: OmegaType = Z - L + N;

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}

// const THRUST_COEFFICIENT: OmegaType = OmegaType::lit("0.00000014");
// const DRAG_COEFFICIENT: OmegaType = OmegaType::lit("0.000002");

pub fn actuate_motors_with_rates(input: &YawPitchRoll<I16F16, I16F16>, raw_lift: I16F16) {
    let n = OmegaType::from_num(input.yaw) * THR_DIV;
    let m = OmegaType::from_num(input.pitch) * DRG_DIV;
    let z = OmegaType::from_num(-input.lift) * DRG_DIV;
    let l = OmegaType::from_num(input.roll) * DRG_DIV;

    let omega_one = (m + m - n - z).max(OmegaType::ZERO);
    let omega_two = (n - l - l - z).max(OmegaType::ZERO);
    let omega_three = (-n - m - m - z).max(OmegaType::ZERO);
    let omega_four = (n + l + l - z).max(OmegaType::ZERO);

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}
