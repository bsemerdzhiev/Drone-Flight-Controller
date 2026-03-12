use my_hdlc::{pc_command::ManualInput, HdlcTransceiver};

use crate::states::state_structures::calibration_state::CalibrationState;

pub struct StateContext<'a> {
    pub calibration_state: &'a mut CalibrationState,
    pub trv: &'a mut HdlcTransceiver,
    pub input_from_controller: &'a mut Option<ManualInput>,
}
