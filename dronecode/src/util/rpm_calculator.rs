use fixed::types::{I16F16, I26F6, I32F0, I32F32, I3F29, I4F28};
use micromath::F32Ext;
use my_hdlc::{
    command::{DebugRpms, DeviceCommand},
    pc_command::ManualInput,
};
use tudelft_quadrupel::{motor, uart::send_bytes};

use crate::util::{
    approx_funcs::{approx_sqrt, approx_sqrt_rpm},
    constants_file::{MAX_LIFT, PITCH_DEGREE, ROLL_DEGREE, YAW_RATE},
    yaw_pitch_roll::YawPitchRoll,
};

//------------------------------------------------------

// chosen by trial and error in Desmos

const MIN_PWM: u16 = 200;

pub const THRESHOLD_LIFT: f32 = 0.05;

fn map_rpm_square_to_pwm(lift_raw_value: f32, rpms_square: &mut [I26F6]) {
    let MAX_RPMS: I16F16 = I16F16::from_num(9800);

    let cur_maxes = I16F16::from_num(motor::get_motor_max());

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k: usize = 0;
    for x in rpms_square {
        let squared_number: I16F16 = I16F16::from_num(approx_sqrt_rpm(*x));

        let rpm_ratio = squared_number / MAX_RPMS;

        pwm_to_set[k] = (cur_maxes * rpm_ratio).to_num::<u16>();

        k += 1;
    }

    //if lift is below threshold, then all motors are off
    if lift_raw_value < THRESHOLD_LIFT {
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

pub fn actuate_motors_with_direct_joystick_input(
    input_from_controller: &YawPitchRoll<I16F16, I16F16>,
    raw_lift: f32,
) {
    let N = I26F6::from_num(input_from_controller.yaw);
    let M = I26F6::from_num(input_from_controller.pitch);
    let Z = I26F6::from_num(-input_from_controller.lift);
    let L = I26F6::from_num(input_from_controller.roll);

    let omega_one: I26F6 = Z + M - N;
    let omega_two: I26F6 = Z + L + N;
    let omega_three: I26F6 = Z - M - N;
    let omega_four: I26F6 = Z - L + N;

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}

// const THRUST_COEFFICIENT: I26F6 = I26F6::lit("0.00000014");
// const DRAG_COEFFICIENT: I26F6 = I26F6::lit("0.000002");
const THR_DIV: I26F6 = I26F6::lit("125000");
const DRG_DIV: I26F6 = I26F6::lit("1785714.286");

pub fn actuate_motors_with_rates(input: &YawPitchRoll<I16F16, I16F16>, raw_lift: f32) {
    let n = I26F6::from_num(input.yaw) * THR_DIV;
    let m = I26F6::from_num(input.pitch) * DRG_DIV;
    let z = I26F6::from_num(-input.lift) * DRG_DIV;
    let l = I26F6::from_num(input.roll) * DRG_DIV;

    let omega_one = (m + m - n - z).max(I26F6::ZERO);
    let omega_two = (n - l - l - z).max(I26F6::ZERO);
    let omega_three = (-n - m - m - z).max(I26F6::ZERO);
    let omega_four = (n + l + l - z).max(I26F6::ZERO);

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}
