use crate::calibration_state::CalibrationState;
use crate::states::safe_mode::*;
use crate::states::FSM_control_trait::FSMControl;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::led::Led::Red;
use tudelft_quadrupel::motor::*;

pub struct FSMPanic;

impl FSMControl for FSMPanic {
    // loop is called every tick
    fn run_control_loop(
        &self,
        calibration_state: &mut CalibrationState,
        command: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl {
        let initial_speed = 100; // change as needed
        let current_speed = get_motors();
        if current_speed.iter().any(|&v| v > initial_speed) {
            set_motors([initial_speed; 4]);
            Red.on();
            return self;
        } else if current_speed[0] == 0 {
            Red.off();
            return &FSMSafe;
        } else {
            // in the case the current maximum is smaller than initial speed
            // equalize all motors and descend from there. Otherwise if motors were set
            // to the initial speed previously it will just keep descending.
            let max_value = current_speed.iter().copied().max().unwrap();
            set_motors([(max_value.saturating_sub(1)); 4]);
            return self;
        }
    }
    fn step(
        &self,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::PanicMode => return self,
            _ => return self, // can only stay in panic or go to safe
        }
    }
}
