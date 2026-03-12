use crate::calibration_state::CalibrationState;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::{panic_mode::FSMPanic, safe_mode::FSMSafe, FSM_control_trait::FSMControl};

use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;

pub struct FSMYaw {
    pub imu_sampler: Box<dyn ImuHandler>,
}

impl FSMControl for FSMYaw {
    fn run_control_loop(
        self: Box<Self>,
        calibration_state: &mut CalibrationState,
        command: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> Box<dyn FSMControl> {
        // read sensor data

        // add to current input

        // output to motors
        return self;
    }
    fn step(
        self: Box<Self>,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic),
            FSMState::SafeMode => Box::new(FSMSafe),
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::YawControl;
    }
}
