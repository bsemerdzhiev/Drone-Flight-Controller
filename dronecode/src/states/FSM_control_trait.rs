use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

use crate::calibration_state::CalibrationState;

pub trait FSMControl {
    fn run_control_loop(
        &self,
        calibration_state: &mut CalibrationState,
        command: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl;
    fn step(
        &self,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl;
    fn get_state(&self) -> FSMState;
}
