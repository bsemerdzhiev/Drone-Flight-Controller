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
pub mod constants_file;

pub const MAX_LIFT: f32 = 25f32;

pub const YAW_RATE: f32 = 375f32;
pub const PITCH_DEGREE: f32 = 2.5f32;
pub const ROLL_DEGREE: f32 = 2.5f32;
