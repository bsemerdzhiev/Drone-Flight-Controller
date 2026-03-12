use alloc::boxed::Box;
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

use crate::states::state_structures::state_context::StateContext;

pub trait FSMControl {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl>;
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl>;
    fn get_state(&self) -> FSMState;
}
