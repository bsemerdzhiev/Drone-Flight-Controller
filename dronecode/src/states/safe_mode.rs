use crate::states::calibration_mode::FSMCalibration;
use crate::states::panic_mode::FSMPanic;
use crate::states::FSM_control_trait::FSMControl;
use crate::{calibration_state::CalibrationState, states::manual_mode::FSMManual};
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};
use tudelft_quadrupel::motor::{self, *};
pub struct FSMSafe;

impl FSMControl for FSMSafe {
    fn run_control_loop(
        &self,
        zero_state: &mut CalibrationState,
        command: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl {
        set_motors([0, 0, 0, 0]);
        return self;
    }
    fn step(
        &self,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return self,
            FSMState::CalibrationMode => {
                if is_throttle_zero() {
                    calibration_state.start_calibration();
                    return &FSMCalibration;
                }
                return self;
            }
            FSMState::FullControlMode => todo!(),
            FSMState::HeightControlMode => todo!(),
            FSMState::ManualMode => return &FSMManual,
            FSMState::PanicMode => return &FSMPanic,
            FSMState::RawSensorsFullControlMode => todo!(),
            FSMState::WirelessMode => todo!(),
            FSMState::YawControl => todo!(),
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
