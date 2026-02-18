use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;
pub trait FSMControl {
    fn run_control_loop(&mut self);
    fn run_safe_mode_cl(&mut self);
}

impl FSMControl for FSMState {
    fn run_control_loop(&mut self) {
        match self {
            FSMState::SafeMode => todo!(),
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

    fn run_safe_mode_cl(&mut self) {
        set_motor_max(0);
    }
}
