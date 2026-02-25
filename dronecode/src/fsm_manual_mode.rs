use my_hdlc::{
    command::{self, Command, DebugRpms},
    pc_command::ManualInput,
};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::control_trait::FSMControl;

const THRUST_COEFFICIENT: i32 = 1;
const DRAG_COEFFICIENT: i32 = 10;

const MAX_BATTERY_VOLTAGE: i32 = 22;
const MOTOR_K_V: i32 = 980;

const MAX_RPM: i32 = MOTOR_K_V * MAX_BATTERY_VOLTAGE;

pub struct FSMManual;

fn my_sqrt(x: i32) -> i32 {
    for i in 0.. {
        if (i * i >= x) {
            return i;
        }
    }
    return 0;
}

fn map_rpm_square_to_pwm(rpms_square: &mut [i32]) {
    let max_allowed_pwm: i32 = motor::get_motor_max() as i32;

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k = 0;
    for x in rpms_square {
        let mut wanted_pwm = 0;
        loop {
            let rhs: i32 = my_sqrt(*x) * max_allowed_pwm / MAX_RPM;
            if (wanted_pwm >= rhs) {
                pwm_to_set[k] = wanted_pwm as u16;
                break;
            }
            wanted_pwm += 1;
        }
        k += 1;
    }
    motor::set_motors(pwm_to_set);
}

impl FSMControl for FSMManual {
    fn run_control_loop(
        &self,
        command: Option<Command>,
        transceiver: &mut my_hdlc::HdlcTransceiver,
    ) {
        let mut input_from_controller: ManualInput = ManualInput::zero();

        if command.is_none() {
            return;
        }

        if let Some(x) = command {
            match x {
                command::Command::ManualInput(manual_input) => {
                    input_from_controller = manual_input;
                }
                _ => {
                    return;
                }
            }
        } else {
            return;
        }

        let Nb: i32 = input_from_controller.get_yaw() * THRUST_COEFFICIENT;
        let Md: i32 = input_from_controller.get_pitch() * DRAG_COEFFICIENT;
        let Zd: i32 = input_from_controller.get_lift() * DRAG_COEFFICIENT;
        let Ld: i32 = input_from_controller.get_roll() * DRAG_COEFFICIENT;

        let four_times_bd: i32 = 4 * DRAG_COEFFICIENT * THRUST_COEFFICIENT;

        let rpm_one: i32 = (-Nb + (2 * Md) - Zd) / (four_times_bd);
        let rpm_two: i32 = (Nb - (2 * Ld) - Zd) / (four_times_bd);
        let rpm_three: i32 = (-Nb - (2 * Md) - Zd) / (four_times_bd);
        let rpm_four: i32 = (-Nb + (2 * Ld) - Zd) / (four_times_bd);

        // let to_write = transceiver.write_structure(&Command::DebugRpms(DebugRpms::new(&[
        //     rpm_one, rpm_two, rpm_three, rpm_four,
        // ])));
        //
        // uart::send_bytes(&to_write.0[0..to_write.1]);

        map_rpm_square_to_pwm(&mut [rpm_one, rpm_two, rpm_three, rpm_four]);
    }

    fn step(&self, next_state: my_hdlc::command::FSMState) -> &dyn FSMControl {
        //TODO:
        match next_state {
            _ => self,
        }
    }
}
