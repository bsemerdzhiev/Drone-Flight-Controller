use crate::control_trait::FSMControl;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;
pub struct FSMSafe;

impl FSMControl for FSMSafe {
    fn run_control_loop(&self) -> &dyn FSMControl {
        set_motor_max(0);
        return self 
    }
    fn step(&self, next_state: my_hdlc::command::FSMState) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::CalibrationMode => todo!(),
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
