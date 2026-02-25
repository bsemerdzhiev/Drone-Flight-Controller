use crate::calibration_state::CalibrationState;
use crate::states::full_control::FSMFullControl;
use crate::states::{safe_mode::FSMSafe, FSM_control_trait::FSMControl};
use my_hdlc::command::FSMState;
use tudelft_quadrupel::mpu::{
    read_raw,
    structs::{Accel, Gyro},
};
pub struct FSMCalibration;

impl FSMControl for FSMCalibration {
    fn run_control_loop(&self, calibration_state: &mut CalibrationState) -> &dyn FSMControl {
        let (accel, gyro) = read_raw().unwrap();
        calibration_state.accumulate_calibration(accel, gyro);
        return self;
    }
    fn step(
        &self,
        next_state: FSMState,
        calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::CalibrationMode => return self,
            FSMState::FullControlMode => {
                calibration_state.finish_calibration();
                return &FSMFullControl;
            }
            FSMState::HeightControlMode => todo!(),
            FSMState::ManualMode => todo!(),
            FSMState::PanicMode => todo!(),
            FSMState::RawSensorsFullControlMode => todo!(),
            FSMState::WirelessMode => todo!(),
            FSMState::YawControl => todo!(),
        }
    }
}
