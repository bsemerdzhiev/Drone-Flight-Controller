use crate::control_trait::FSMControl;
use crate::fsm_safe_mode::*;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;
use tudelft_quadrupel::led::Led::Green;

pub struct FSMPanic;

impl FSMControl for FSMPanic {
    // loop is called every tick
    fn run_control_loop(&self) {
        let initial_speed = 100; // change as needed
        let current_speed = get_motor_max();
        if current_speed > initial_speed {
            set_motor_max(initial_speed);
            Green.toggle();
        } else if current_speed == 0 {
            Green.toggle();
            // should go to safe mode
        } else {
            set_motor_max(current_speed - 1);
        }
    }
    fn step(&self, next_state: FSMState) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::PanicMode => return self,
            _ => return self, // can only stay in panic or go to safe
        }
    }
}


