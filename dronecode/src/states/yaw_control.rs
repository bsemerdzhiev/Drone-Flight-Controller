use crate::calibration_state::CalibrationState;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_context::StateContext;
use crate::states::{fsm_control_trait::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe};

use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;

pub struct FSMYaw {
    pub imu_sampler: Box<dyn ImuHandler>,
}

impl FSMControl for FSMYaw {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data

        // add to current input

        // output to motors
        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::SafeMode => Box::new(FSMSafe {}),
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::YawControl;
    }
}
