use my_hdlc::{
    command::{self, DebugRpms, DeviceCommand, FSMState},
    pc_command::ManualInput,
    HdlcTransceiver,
};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::{
    calibration_state::CalibrationState,
    states::{panic_mode::FSMPanic, safe_mode::FSMSafe, FSM_control_trait::FSMControl},
};

const THRUST_COEFFICIENT: f32 = 1e-2;
const DRAG_COEFFICIENT: f32 = 1e-3;

const MAX_BATTERY_VOLTAGE: i32 = 22;
const MOTOR_K_V: i32 = 980;

const MAX_RPM: i32 = MOTOR_K_V * MAX_BATTERY_VOLTAGE;

const MAX_POSSIBLE_PWM: i32 = 5000;

const LINEAR_FACTOR: u16 = 10;

pub struct FSMManual;

fn my_sqrt(x: i32) -> i32 {
    let mut to_return = 0;
    for i in 0..10000 {
        if (i * i >= x) {
            to_return = i;
            break;
        }
    }
    return to_return;
}

fn map_rpm_square_to_pwm(rpms_square: &mut [i32], transceiver: &mut my_hdlc::HdlcTransceiver) {
    let max_allowed_pwm: i32 = MAX_POSSIBLE_PWM; //motor::get_motor_max() as i32;

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut k: usize = 0;
    for x in rpms_square {
        let squared_number: u16 = my_sqrt(*x) as u16;
        // let rhs: u16 = squared_number * LINEAR_FACTOR;
        let rhs = squared_number as u16;
        pwm_to_set[k] = rhs;

        k += 1;
    }

    motor::set_motors(pwm_to_set);
}

impl FSMControl for FSMManual {
    fn run_control_loop(
        &self,
        calibration_state: &mut crate::calibration_state::CalibrationState,
        input_from_controller: ManualInput,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl {
        let Nb: f32 = input_from_controller.get_yaw() as f32 * THRUST_COEFFICIENT;
        let Md: f32 = input_from_controller.get_pitch() as f32 * DRAG_COEFFICIENT;
        let Zd: f32 = -input_from_controller.get_lift() as f32 * DRAG_COEFFICIENT;
        let Ld: f32 = input_from_controller.get_roll() as f32 * DRAG_COEFFICIENT;

        let four_times_bd: f32 = 4.0 * DRAG_COEFFICIENT * THRUST_COEFFICIENT;

        let rpm_one: i32 = (((-Nb - (2.0 * Md) - Zd) / (four_times_bd)) as i32).max(0);
        let rpm_two: i32 = (((Nb - (2.0 * Ld) - Zd) / (four_times_bd)) as i32).max(0);
        let rpm_three: i32 = (((-Nb + (2.0 * Md) - Zd) / (four_times_bd)) as i32).max(0);
        let rpm_four: i32 = (((Nb + (2.0 * Ld) - Zd) / (four_times_bd)) as i32).max(0);

        // let to_write = transceiver.write_structure(&Command::ManualInput(input_from_controller));
        // let to_write = transceiver.write_structure(&Command::DebugRpms(DebugRpms::new(&[
        // rpm_one as i32,
        // rpm_two as i32,
        // rpm_three as i32,
        // rpm_four as i32,
        // ])));

        // uart::send_bytes(&to_write.0[0..to_write.1]);
        map_rpm_square_to_pwm(&mut [rpm_one, rpm_two, rpm_three, rpm_four], my_hdlc);
        self
    }

    fn step(
        &self,
        next_state: my_hdlc::command::FSMState,
        calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::CalibrationMode => todo!(),
            FSMState::FullControlMode => todo!(),
            FSMState::HeightControlMode => todo!(),
            FSMState::ManualMode => &FSMManual,
            FSMState::PanicMode => return &FSMPanic,
            FSMState::RawSensorsFullControlMode => todo!(),
            FSMState::WirelessMode => todo!(),
            FSMState::YawControl => todo!(),
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::ManualMode;
    }
}
