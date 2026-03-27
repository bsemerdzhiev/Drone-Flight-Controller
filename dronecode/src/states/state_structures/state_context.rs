use alloc::boxed::Box;
use my_hdlc::{pc_command::ManualInput, HdlcTransceiver};

use crate::states::state_structures::calibration_state::CalibrationState;

pub struct StateContext<'a> {
    pub calibration_state: &'a mut CalibrationState,
    pub trv: &'a mut Box<HdlcTransceiver>,
    pub input_from_controller: &'a mut Option<ManualInput>,
    pub flash_head: &'a mut u32,
    pub flash_tail: &'a mut u32,
}
