use crate::calibration_state::CalibrationState;
use alloc::boxed::Box;
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

pub trait FSMControl {
    fn run_control_loop(
        self: Box<Self>,
        calibration_state: &mut CalibrationState,
        command: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> Box<dyn FSMControl>;
    fn step(
        self: Box<Self>,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> Box<dyn FSMControl>;
    fn get_state(&self) -> FSMState;
}
