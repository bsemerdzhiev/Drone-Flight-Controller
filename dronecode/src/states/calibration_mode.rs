use crate::control_trait::FSMControl;
use crate::sensor_state::SensorState;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;
pub struct FSMCalibration;

impl FSMControl for FSMCalibration {
    fn run_control_loop(&self, zero_state: &mut SensorState) {
        zero_state.update_quaternion();
        zero_state.update_raw_data();
    }
    fn step(&self, next_state: my_hdlc::command::FSMState) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::CalibrationMode => return &FSMCalibration,
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
