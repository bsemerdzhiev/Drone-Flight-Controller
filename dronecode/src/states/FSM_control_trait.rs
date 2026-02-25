use my_hdlc::command::FSMState;

use crate::sensor_state::SensorState;

pub trait FSMControl {
    fn run_control_loop(&self, zero_state: &mut SensorState) -> &dyn FSMControl;
    // fn run_safe_mode_cl(& self);
    fn step(&self, next_state: FSMState) -> &dyn FSMControl;
}

// impl FSMControl for FSMState {
//     fn run_control_loop(&mut self) {
//         match self {
//             FSMState::SafeMode => self.run_safe_mode_cl(),
//             FSMState::CalibrationMode => todo!(),
//             FSMState::FullControlMode => todo!(),
//             FSMState::HeightControlMode => todo!(),
//             FSMState::ManualMode => todo!(),
//             FSMState::PanicMode => todo!(),
//             FSMState::RawSensorsFullControlMode => todo!(),
//             FSMState::WirelessMode => todo!(),
//             FSMState::YawControl => todo!(),
//         }
//     }

//     fn run_safe_mode_cl(&mut self) {
//         set_motor_max(0);
//     }
// }
