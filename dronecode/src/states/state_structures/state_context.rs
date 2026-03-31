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
    pub flash_head: &'a mut usize,
    pub flash_tail: &'a mut usize,

    pub time_for_main_loop: f32,

    pub pid_info: &'a mut Box<PIDInfo>,
}

pub struct PIDInfo {
    pub selected_height: f32,
}
