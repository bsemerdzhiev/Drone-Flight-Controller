use crate::filters::dmp_readings::DmpReadings;
use crate::states::full_control::FSMFullControl;
use crate::states::manual_mode::FSMManual;
use crate::states::panic_mode::FSMPanic;
use crate::states::state_structures::state_context::StateContext;
use crate::states::yaw_control::FSMYaw;
use crate::states::{fsm_base_class::FSMControl, safe_mode::FSMSafe};

use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::mpu::{
    read_raw,
    structs::{Accel, Gyro},
};

pub struct FSMCalibration {}

impl FSMControl for FSMCalibration {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        let (accel, gyro) = read_raw().unwrap();

        // read new sample
        ctx.calibration_state.read_new_sample(accel, gyro);

        if ctx.calibration_state.should_finish() {
            ctx.calibration_state.finalize_calibration();

            return Box::new(FSMSafe {});
        }
        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::SafeMode => Box::new(FSMSafe {}),
            _ => return self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::CalibrationMode;
    }
}
