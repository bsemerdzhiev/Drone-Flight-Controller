use crate::sensor_state::SensorState;
use crate::states::{safe_mode::FSMSafe, FSM_control_trait::FSMControl};
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;

pub struct FSMCalibration;

impl FSMControl for FSMCalibration {
    fn run_control_loop(&self, zero_state: &mut SensorState) -> &dyn FSMControl {
        zero_state.update_quaternion();
        zero_state.update_raw_data();
        return self;
    }
    fn step(&self, next_state: FSMState) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::CalibrationMode => return self,
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
