use alloc::boxed::Box;
use fixed::types::{I16F16, I26F6, I2F30, I4F28, I8F24};
use my_hdlc::{
    pc_command::{ManualDroneInput, ManualDroneTrims},
    HdlcTransceiver,
};

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

    pub trim_input: &'a mut ManualDroneTrims,

    pub calibration_state: &'a mut CalibrationState<I8F24, I8F24>,
    pub input_as_ypr: &'a mut YawPitchRoll<I16F16, I16F16>,

    pub flash_head: &'a mut usize,
    pub flash_tail: &'a mut usize,

    pub time_for_main_loop: i32,

    pub pid_info: &'a mut Box<PIDInfo>,

    pub is_wireless: &'a mut bool,
}

pub struct PIDInfo {
    pub selected_height: f32,
}
