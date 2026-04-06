#[cfg(target_arch = "arm")]
pub mod rpm_calculator;

#[cfg(target_arch = "arm")]
pub mod yaw_pitch_roll;

#[cfg(target_arch = "arm")]
pub mod approx_funcs;

#[cfg(target_arch = "arm")]
pub mod pid_controller;

#[cfg(target_arch = "arm")]
pub mod axis;

#[cfg(target_arch = "arm")]
pub mod ble_communication;

pub const MAX_LIFT: i32 = 15i32;

pub const YAW_RATE: i32 = 120i32;
pub const PITCH_DEGREE: i32 = 30i32;
pub const ROLL_DEGREE: i32 = 30i32;
