use my_hdlc::{pc_command::ManualInput, HdlcTransceiver};

use crate::states::FSM_control_trait::FSMControl;
pub struct FSMFullControl;

impl FSMControl for FSMFullControl {
    fn run_control_loop(
        &self,
        calibration_state: &mut crate::calibration_state::CalibrationState,
        command: ManualInput,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl {
        todo!();
    }
    fn step(
        &self,
        next_state: my_hdlc::command::FSMState,
        calibration_state: &mut crate::calibration_state::CalibrationState,
    ) -> &dyn FSMControl {
        todo!();
    }
}
