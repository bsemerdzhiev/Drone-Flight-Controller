use alloc::boxed::Box;
use my_hdlc::{pc_command::ManualInput, HdlcTransceiver};

use crate::{
    filters::{
        dmp_readings::DmpReadings, kalman_filter::KalmanFilter, pressure_filter::PressureSensor,
    },
    states::state_structures::calibration_state::CalibrationState,
};

pub struct StateContext<'a> {
    pub kalman_position: KalmanFilter,
    pub pressure_sensor_filter: &'a mut PressureSensor,
    pub dmp_filter: DmpReadings,

    pub calibration_state: &'a mut CalibrationState,
    pub trv: &'a mut Box<HdlcTransceiver>,
    pub input_from_controller: &'a mut Option<ManualInput>,
    pub flash_head: &'a mut u32,
    pub flash_tail: &'a mut u32,
}
