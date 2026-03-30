use micromath::F32Ext;
use my_hdlc::{
    command::{DebugRpms, DeviceCommand},
    pc_command::ManualInput,
};
use tudelft_quadrupel::{motor, uart::send_bytes};

use crate::util::{yaw_pitch_roll::YawPitchRoll, MAX_LIFT, PITCH_DEGREE, ROLL_DEGREE, YAW_RATE};

//------------------------------------------------------

const THRUST_COEFFICIENT: f32 = 14e-8;
const DRAG_COEFFICIENT: f32 = 2e-6;

const MIN_PWM: u16 = 50;

const MAX_RPMS: f32 = (980 * 10) as f32;

const THRESHOLD_LIFT: f32 = 0.2;

fn map_rpm_square_to_pwm(lift_raw_value: f32, rpms_square: &mut [f32]) {
    let cur_maxes = motor::get_motor_max();

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut all_zero: bool = true;

    let mut k: usize = 0;
    for x in rpms_square {
        let squared_number: f32 = f32::sqrt(*x as f32);

        let rpm_ratio = squared_number / MAX_RPMS;

        pwm_to_set[k] = (cur_maxes as f32 * rpm_ratio) as u16;

        if pwm_to_set[k] != 0 {
            all_zero = false;
        }
        k += 1;
    }

    if lift_raw_value < THRESHOLD_LIFT {
        for cur_motor_rpm in &mut pwm_to_set {
            *cur_motor_rpm = 0;
        }
    } else if !all_zero {
        for cur_motor_rpm in &mut pwm_to_set {
            *cur_motor_rpm = MIN_PWM.max(*cur_motor_rpm);
        }
    }

    motor::set_motors(pwm_to_set);
}

pub fn actuate_motors_with_direct_joystick_input(input_from_controller: &ManualInput) {
    let N = YAW_RATE * input_from_controller.get_yaw();
    let M = PITCH_DEGREE * input_from_controller.get_pitch();
    let Z = MAX_LIFT * -input_from_controller.get_lift();
    let L = ROLL_DEGREE * input_from_controller.get_roll();

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
    let Nb: f32 = input_from_controller.yaw * THRUST_COEFFICIENT;
    let Md: f32 = input_from_controller.pitch * DRAG_COEFFICIENT;
    let Zd: f32 = -input_from_controller.lift * DRAG_COEFFICIENT;
    let Ld: f32 = input_from_controller.roll * DRAG_COEFFICIENT;

    let four_times_bd: f32 = 4.0 * DRAG_COEFFICIENT * THRUST_COEFFICIENT;

    let omega_one: f32 = ((-Nb + (2.0 * Md) - Zd) / (four_times_bd)).max(0.0);
    let omega_two: f32 = ((Nb - (2.0 * Ld) - Zd) / (four_times_bd)).max(0.0);
    let omega_three: f32 = ((-Nb - (2.0 * Md) - Zd) / (four_times_bd)).max(0.0);
    let omega_four: f32 = ((Nb + (2.0 * Ld) - Zd) / (four_times_bd)).max(0.0);

    map_rpm_square_to_pwm(
        raw_lift,
        &mut [omega_one, omega_two, omega_three, omega_four],
    );
}
