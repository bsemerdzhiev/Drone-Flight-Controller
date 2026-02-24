use my_hdlc::{command::Command, pc_command::ManualInput};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::control_trait::FSMControl;

const THRUST_COEFFICIENT: i32 = 100;
const DRAG_COEFFICIENT: i32 = 1000;

const MAX_BATTERY_VOLTAGE: i32 = 22;
const MOTOR_K_V: i32 = 980;

const PWM_MAX_VALUE: i32 = 1000;
const MAX_RPM: i32 = MOTOR_K_V * MAX_BATTERY_VOLTAGE;

const MAX_RATIO: i32 = (PWM_MAX_VALUE / MAX_RPM) * (PWM_MAX_VALUE / MAX_RPM);

pub struct FSMManual;

fn map_rpm_square_to_pwm(rpms_square: &mut [i32]) {
    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k = 0;
    for x in rpms_square {
        let mut wanted_pwm = 0;
        loop {
            if (wanted_pwm * wanted_pwm > (*x * MAX_RATIO)) {
                pwm_to_set[k] = (wanted_pwm - 1) as u16;
                break;
            }
            wanted_pwm += 1;
        }
        k += 1;
    }
    motor::set_motors(pwm_to_set);
}

impl FSMControl for FSMManual {
    fn run_control_loop(&self, command: &Option<Command>) {
        uart::send_bytes("Test\n".as_bytes());
        return;
        // let Nb: i32 = input_from_controller.get_yaw() * THRUST_COEFFICIENT;
        // let Md: i32 = input_from_controller.get_pitch() * DRAG_COEFFICIENT;
        // let Zd: i32 = input_from_controller.get_lift() * DRAG_COEFFICIENT;
        // let Ld: i32 = input_from_controller.get_roll() * DRAG_COEFFICIENT;
        //
        // let four_times_bd: i32 = 4 * DRAG_COEFFICIENT * THRUST_COEFFICIENT;
        //
        // let rpm_one: i32 = (-Nb + (2 * Md) - Zd) / (four_times_bd);
        // let rpm_two: i32 = (Nb - (2 * Ld) - Zd) / (four_times_bd);
        // let rpm_three: i32 = (-Nb - (2 * Md) - Zd) / (four_times_bd);
        // let rpm_four: i32 = (-Nb + (2 * Ld) - Zd) / (four_times_bd);
        //
        // map_rpm_square_to_pwm(&mut [rpm_one, rpm_two, rpm_three, rpm_four]);
    }

    fn step(&self, next_state: my_hdlc::command::FSMState) -> &dyn FSMControl {
        //TODO:
        match next_state {
            _ => self,
        }
    }
}
