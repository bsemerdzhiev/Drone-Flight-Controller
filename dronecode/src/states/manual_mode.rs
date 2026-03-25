use crate::{
    states::state_structures::state_context::StateContext,
    util::rpm_calculator::actuate_motors_with_rates,
};
use alloc::boxed::Box;
use my_hdlc::{
    command::{self, DebugRpms, DeviceCommand, FSMState},
    pc_command::ManualInput,
    HdlcTransceiver,
};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::states::{fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe};

pub struct FSMManual {}

impl FSMControl for FSMManual {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // check if there is a new command from the controller to run
        if ctx.input_from_controller.is_none() {
            return self;
        }
        actuate_motors_with_rates(&ctx.input_from_controller.as_ref().unwrap(), ctx.trv);

        // consume the command and set it to None
        *ctx.input_from_controller = None;
        self
    }

    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => return Box::new(FSMPanic {}),
            _ => return self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::ManualMode;
    }
}
