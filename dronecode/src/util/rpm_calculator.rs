use micromath::F32Ext;
use my_hdlc::{
    command::{DebugRpms, DeviceCommand},
    pc_command::ManualInput,
};
use tudelft_quadrupel::{motor, uart::send_bytes};

use crate::util::{
    approx_funcs::approx_sqrt,
    constants_file::{ChosenFixedPointType, MAX_LIFT, PITCH_DEGREE, ROLL_DEGREE, YAW_RATE},
    yaw_pitch_roll::YawPitchRoll,
};

//------------------------------------------------------

// chosen by trial and error in Desmos
const THRUST_COEFFICIENT: ChosenFixedPointType = ChosenFixedPointType::lit("14e-8");
const DRAG_COEFFICIENT: ChosenFixedPointType = ChosenFixedPointType::lit("2e-6");

const MIN_PWM: u16 = 50;
const MAX_RPMS: ChosenFixedPointType = ChosenFixedPointType::lit("9800");

pub const THRESHOLD_LIFT: f32 = 0.1;

fn map_rpm_square_to_pwm(lift_raw_value: f32, rpms_square: &mut [ChosenFixedPointType]) {
    let cur_maxes = ChosenFixedPointType::from_num(motor::get_motor_max());

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k: usize = 0;
    for x in rpms_square {
        let squared_number: ChosenFixedPointType = approx_sqrt(*x);

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

pub fn actuate_motors_with_direct_joystick_input(input_from_controller: &ManualInput) {
    let N = YAW_RATE * ChosenFixedPointType::from_num(input_from_controller.get_yaw());
    let M = PITCH_DEGREE * ChosenFixedPointType::from_num(input_from_controller.get_pitch());
    let Z = MAX_LIFT * ChosenFixedPointType::from_num(-input_from_controller.get_lift());
    let L = ROLL_DEGREE * ChosenFixedPointType::from_num(input_from_controller.get_roll());

    let raw_lift: f32 = input_from_controller.get_lift();

    let omega_one = Z + M - N;
    let omega_two = Z + L + N;
    let omega_three = Z - M - N;
    let omega_four = Z - L + N;

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}

pub fn actuate_motors_with_rates(input_from_controller: &YawPitchRoll, raw_lift: f32) {
    let Nb: ChosenFixedPointType = input_from_controller.yaw * THRUST_COEFFICIENT;
    let Md: ChosenFixedPointType = input_from_controller.pitch * DRAG_COEFFICIENT;
    let Zd: ChosenFixedPointType = -input_from_controller.lift * DRAG_COEFFICIENT;
    let Ld: ChosenFixedPointType = input_from_controller.roll * DRAG_COEFFICIENT;

    let FOUR_TIMES_BD: ChosenFixedPointType =
        ChosenFixedPointType::lit("4") * DRAG_COEFFICIENT * THRUST_COEFFICIENT;

    let omega_one: ChosenFixedPointType =
        ((-Nb + (2 * Md) - Zd) / (FOUR_TIMES_BD)).max(ChosenFixedPointType::from_num(0));
    let omega_two: ChosenFixedPointType =
        ((Nb - (2 * Ld) - Zd) / (FOUR_TIMES_BD)).max(ChosenFixedPointType::from_num(0));
    let omega_three: ChosenFixedPointType =
        ((-Nb - (2 * Md) - Zd) / (FOUR_TIMES_BD)).max(ChosenFixedPointType::from_num(0));
    let omega_four: ChosenFixedPointType =
        ((Nb + (2 * Ld) - Zd) / (FOUR_TIMES_BD)).max(ChosenFixedPointType::from_num(0));

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}
