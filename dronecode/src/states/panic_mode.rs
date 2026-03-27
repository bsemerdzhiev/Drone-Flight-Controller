use crate::states::fsm_base_class::FSMControl;
use crate::states::safe_mode::*;
use crate::states::state_structures::state_context::StateContext;
use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::led::Led::Red;
use tudelft_quadrupel::motor::*;

pub struct FSMPanic {}

impl FSMControl for FSMPanic {
    // loop is called every tick
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        let current_speed = get_motors();
        const DESCENT_STEP: u16 = 2;

        Red.on();
        let mut avg_speed: u16 = 0;
        for i in current_speed {
            avg_speed += i;
        }
        avg_speed /= 4;

        if current_speed.iter().any(|&v| v != avg_speed) {
            set_motors([avg_speed; 4]);
            return self;
        } else if current_speed[0] == 0 {
            return Box::new(FSMSafe {});
        } else {
            // all motors are equalized

            let new_speed: u16 = (avg_speed.saturating_sub(DESCENT_STEP)).max(0u16);

            set_motors([new_speed; 4]);
            return self;
        }
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::SafeMode => {
                return Box::new(FSMSafe {});
            }
            FSMState::PanicMode => return self,
            _ => return self, // can only stay in panic or go to safe
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::PanicMode;
    }
}
