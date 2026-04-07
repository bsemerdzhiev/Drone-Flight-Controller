use core::{default, time::Duration};

use alloc::boxed::Box;
use fixed::types::{I16F16, I26F6, I2F30, I4F28, I8F24};
use my_hdlc::{
    command::FSMState,
    pc_command::{ManualDroneInput, ManualDroneTrimsEnums},
    HdlcTransceiver, STUFFED_MESSAGE_SIZE,
};
use tudelft_quadrupel::time::Instant;

use crate::{
    filters::{
        dmp_readings::DmpReadings, kalman_filter::KalmanFilter, pressure_filter::PressureSensor,
    },
    states::state_structures::calibration_state::CalibrationState,
    util::{pid_controller::PIDController, yaw_pitch_roll::YawPitchRoll},
};

pub struct StateContext<'a> {
    pub kalman_position: KalmanFilter,
    pub pressure_sensor_filter: &'a mut PressureSensor,
    pub dmp_filter: DmpReadings,

    pub pid_controller: PIDController<I16F16, I16F16>,

    pub trv: &'a mut Box<HdlcTransceiver>,

    pub calibration_state: &'a mut CalibrationState<I16F16, I16F16>,
    pub input_as_ypr: &'a mut YawPitchRoll<I16F16, I16F16>,

    pub flash_head: usize,
    pub flash_tail: usize,

    pub time_for_main_loop: i32,

    pub pid_info: &'a mut Box<PIDInfo>,

    pub is_wireless: bool,
    pub wireless_log: SendMessageWirelessly,

    pub curent_state: FSMState,
    pub dt: Duration,
}

pub struct SendMessageWirelessly {
    pub message_ind: usize,
    pub message_part: usize,
    pub forced_message: bool,

    pub stored_msg: [u8; STUFFED_MESSAGE_SIZE],
    pub message_len: usize,

    pub last_sent_logged_message: Instant,
}

impl Default for SendMessageWirelessly {
    fn default() -> Self {
        Self {
            message_ind: 0usize,
            message_part: 0usize,
            forced_message: false,
            stored_msg: [0u8; STUFFED_MESSAGE_SIZE],
            message_len: 0usize,

            last_sent_logged_message: Instant::now(),
        }
    }
}

pub struct PIDInfo {
    pub selected_height: f32,
}
