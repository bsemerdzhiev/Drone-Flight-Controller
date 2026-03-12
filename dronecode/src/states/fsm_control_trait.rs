use crate::{calibration_state::CalibrationState, states::state_context::StateContext};
use alloc::boxed::Box;
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

pub trait FSMControl {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl>;
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl>;
    fn get_state(&self) -> FSMState;
}
