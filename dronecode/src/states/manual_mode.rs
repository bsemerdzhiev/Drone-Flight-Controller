use crate::{states::state_context::StateContext, util::rpm_calculator::map_rms};

use alloc::boxed::Box;
use my_hdlc::{
    command::{self, DebugRpms, DeviceCommand, FSMState},
    pc_command::ManualInput,
    HdlcTransceiver,
};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::{
    calibration_state::CalibrationState,
    states::{fsm_control_trait::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe},
};

pub struct FSMManual {}

impl FSMControl for FSMManual {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        if ctx.input_from_controller.is_none() {
            return self;
        }
        map_rms(&ctx.input_from_controller.as_ref().unwrap(), ctx.trv);
        *ctx.input_from_controller = None;

        self
    }

    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::SafeMode => return Box::new(FSMSafe {}),
            FSMState::PanicMode => return Box::new(FSMPanic {}),
            _ => return self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::ManualMode;
    }
}
