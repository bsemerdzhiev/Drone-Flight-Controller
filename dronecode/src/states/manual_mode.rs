use crate::util::rpm_calculator::map_rms;

use alloc::boxed::Box;
use my_hdlc::{
    command::{self, DebugRpms, DeviceCommand, FSMState},
    pc_command::ManualInput,
    HdlcTransceiver,
};
use tudelft_quadrupel::{cortex_m::prelude::_embedded_hal_serial_Read, motor, uart};

use crate::{
    calibration_state::CalibrationState,
    states::{panic_mode::FSMPanic, safe_mode::FSMSafe, FSM_control_trait::FSMControl},
};

pub struct FSMManual;

impl FSMControl for FSMManual {
    fn run_control_loop(
        self: Box<Self>,
        calibration_state: &mut crate::calibration_state::CalibrationState,
        input_from_controller: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> Box<dyn FSMControl> {
        if !*has_received_input {
            return self;
        }
        map_rms(&input_from_controller, my_hdlc);
        *has_received_input = false;
        self
    }

    fn step(
        self: Box<Self>,
        next_state: my_hdlc::command::FSMState,
        calibration_state: &mut CalibrationState,
    ) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::SafeMode => return Box::new(FSMSafe),
            FSMState::CalibrationMode => return self,
            FSMState::FullControlMode => return self,
            FSMState::HeightControlMode => return self,
            FSMState::ManualMode => return self,
            FSMState::RawSensorsFullControlMode => return self,
            FSMState::WirelessMode => return self,
            FSMState::YawControl => return self,
            FSMState::PanicMode => return Box::new(FSMPanic),
            _ => return self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::ManualMode;
    }
}
