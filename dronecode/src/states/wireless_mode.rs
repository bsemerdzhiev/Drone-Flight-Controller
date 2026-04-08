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
        ctx.is_wireless ^= true;
        ctx.wireless_log.forced_message = true;

        ctx.wireless_log.message_part = 0;
        // set the message ind to 1 so that all messages that will be generated from 1 to 7 can clear the previous bits in
        // the buffer
        ctx.wireless_log.message_ind = 1;

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
