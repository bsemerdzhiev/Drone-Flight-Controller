use crate::states::calibration_mode::FSMCalibration;
use crate::states::fsm_base_class::FSMControl;
use crate::states::panic_mode::FSMPanic;
use crate::states::state_context::StateContext;
use crate::{calibration_state::CalibrationState, states::manual_mode::FSMManual};
use alloc::boxed::Box;
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};
use tudelft_quadrupel::led::Red;
use tudelft_quadrupel::motor::{self, *};

pub struct FSMSafe {}

impl FSMControl for FSMSafe {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        set_motors([0, 0, 0, 0]);
        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        if next_state != FSMState::SafeMode {
            Red.off();
        }
        match next_state {
            FSMState::CalibrationMode => {
                if is_throttle_zero() {
                    ctx.calibration_state.start_calibration();
                    return Box::new(FSMCalibration {});
                }
                return self;
            }
            FSMState::ManualMode => return Box::new(FSMManual {}),
            FSMState::PanicMode => return Box::new(FSMPanic {}),
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::SafeMode;
    }
}

fn is_throttle_zero() -> bool {
    let speed = get_motors();
    return speed[0] == 0 && speed[1] == 0 && speed[2] == 0 && speed[3] == 0;
}
