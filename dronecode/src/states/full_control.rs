use my_hdlc::pc_command::ManualInput;

use crate::states::FSM_control_trait::FSMControl;
pub struct FSMFullControl;

impl FSMControl for FSMFullControl {
    fn step(
        &self,
        next_state: my_hdlc::command::FSMState,
        calibration_state: &mut crate::calibration_state::CalibrationState,
    ) -> &dyn FSMControl {
        todo!();
    }
    fn run_control_loop(
        &self,
        calibration_state: &mut crate::calibration_state::CalibrationState,
        command: ManualInput,
    ) -> &dyn FSMControl {
        todo!();
    }
}
