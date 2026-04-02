use alloc::boxed::Box;
use fixed::types::{I16F16, I26F6, I2F30, I4F28};
use my_hdlc::{pc_command::ManualInput, HdlcTransceiver};

use crate::{
    filters::{
        dmp_readings::DmpReadings, kalman_filter::KalmanFilter, pressure_filter::PressureSensor,
    },
    states::state_structures::calibration_state::CalibrationState,
    util::yaw_pitch_roll::YawPitchRoll,
};

pub struct StateContext<'a> {
    pub kalman_position: KalmanFilter,
    pub pressure_sensor_filter: &'a mut PressureSensor,
    pub dmp_filter: DmpReadings,

    pub trv: &'a mut Box<HdlcTransceiver>,

    pub input_from_controller: &'a mut ManualInput,

    pub calibration_state: &'a mut CalibrationState<I4F28, I4F28>,
    pub input_as_ypr: &'a mut YawPitchRoll<I16F16, I16F16>,

    pub flash_head: &'a mut usize,
    pub flash_tail: &'a mut usize,

    pub time_for_main_loop: f32,

    pub pid_info: &'a mut Box<PIDInfo>,
}

pub struct PIDInfo {
    pub selected_height: f32,
}
