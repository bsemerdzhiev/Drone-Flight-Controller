use my_hdlc::{
    command::{self, Command, DebugRpms},
    pc_command::ManualInput,
};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::control_trait::FSMControl;

const THRUST_COEFFICIENT: f32 = 1e-5;
const DRAG_COEFFICIENT: f32 = 1e-7;

const MAX_BATTERY_VOLTAGE: i32 = 22;
const MOTOR_K_V: i32 = 980;

const MAX_RPM: i32 = MOTOR_K_V * MAX_BATTERY_VOLTAGE;

const MAX_POSSIBLE_PWM: i32 = 5000;

pub struct FSMManual;

fn my_sqrt(x: i32) -> i32 {
    for i in 0.. {
        if (i * i >= x) {
            return i;
        }
    }
    return 0;
}

fn map_rpm_square_to_pwm(rpms_square: &mut [i32], transceiver: &mut my_hdlc::HdlcTransceiver) {
    let max_allowed_pwm: i32 = MAX_POSSIBLE_PWM; //motor::get_motor_max() as i32;

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k = 0;
    for x in rpms_square {
        let mut wanted_pwm = 0;
        let squared_number = my_sqrt(*x);
        loop {
            let rhs: i32 = (squared_number * max_allowed_pwm) / MAX_RPM;
            if (wanted_pwm >= rhs) {
                pwm_to_set[k] = wanted_pwm as u16;
                break;
            }
            wanted_pwm += 1;
        }
        k += 1;
    }
    // let to_write = transceiver.write_structure(&Command::DebugRpms(DebugRpms::new(&pwm_to_set)));

    // uart::send_bytes(&to_write.0[0..to_write.1]);

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

        let Nb: f32 = input_from_controller.get_yaw() as f32 * THRUST_COEFFICIENT;
        let Md: f32 = input_from_controller.get_pitch() as f32 * DRAG_COEFFICIENT;
        let Zd: f32 = input_from_controller.get_lift() as f32 * DRAG_COEFFICIENT;
        let Ld: f32 = input_from_controller.get_roll() as f32 * DRAG_COEFFICIENT;

        let four_times_bd: f32 = 4.0 * DRAG_COEFFICIENT * THRUST_COEFFICIENT;

        let rpm_one: i32 = ((-Nb + (2.0 * Md) - Zd) / (four_times_bd)) as i32;
        let rpm_two: i32 = ((Nb - (2.0 * Ld) - Zd) / (four_times_bd)) as i32;
        let rpm_three: i32 = ((-Nb - (2.0 * Md) - Zd) / (four_times_bd)) as i32;
        let rpm_four: i32 = ((-Nb + (2.0 * Ld) - Zd) / (four_times_bd)) as i32;

        map_rpm_square_to_pwm(&mut [rpm_one, rpm_two, rpm_three, rpm_four], transceiver);
    }

    fn step(&self, next_state: my_hdlc::command::FSMState) -> &dyn FSMControl {
        //TODO:
        match next_state {
            _ => self,
        }
    }
}
