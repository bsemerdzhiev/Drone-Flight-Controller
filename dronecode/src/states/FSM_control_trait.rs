use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

use crate::calibration_state::CalibrationState;

pub trait FSMControl {
    fn run_control_loop(
        &self,
        calibration_state: &mut CalibrationState,
        command: ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl;
    // fn run_safe_mode_cl(& self);
    fn step(
        &self,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl;
}

// impl FSMControl for FSMState {
//     fn run_control_loop(&mut self) {
//         match self {
//             FSMState::SafeMode => self.run_safe_mode_cl(),
//             FSMState::CalibrationMode => todo!(),
//             FSMState::FullControlMode => todo!(),
//             FSMState::HeightControlMode => todo!(),
//             FSMState::ManualMode => todo!(),
//             FSMState::PanicMode => todo!(),
//             FSMState::RawSensorsFullControlMode => todo!(),
//             FSMState::WirelessMode => todo!(),
//             FSMState::YawControl => todo!(),
//         }
//     }

//     fn run_safe_mode_cl(&mut self) {
//         set_motor_max(0);
//     }
// }
