use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::led::Led::Yellow;

use crate::states::{
    fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe,
    state_structures::state_context::StateContext,
};

pub struct FSMWireless {}

impl FSMControl for FSMWireless {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        *ctx.is_wireless ^= true;
        Yellow.toggle();

        return Box::new(FSMPanic {});
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            _ => return self, // can only stay in this mode, should transition by itself
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::WirelessMode;
    }
}
