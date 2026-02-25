use crate::calibration_state::CalibrationState;
use crate::states::calibration_mode::FSMCalibration;
use crate::states::FSM_control_trait::FSMControl;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;
pub struct FSMSafe;

impl FSMControl for FSMSafe {
    fn run_control_loop(&self, zero_state: &mut CalibrationState) -> &dyn FSMControl {
        set_motor_max(0);
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
                calibration_state.start_calibration();
                return &FSMCalibration;
            }
            FSMState::FullControlMode => todo!(),
            FSMState::HeightControlMode => todo!(),
            FSMState::ManualMode => todo!(),
            FSMState::PanicMode => todo!(),
            FSMState::RawSensorsFullControlMode => todo!(),
            FSMState::WirelessMode => todo!(),
            FSMState::YawControl => todo!(),
        }
    }
}
