#[cfg(target_arch = "arm")]
pub mod rpm_calculator;

#[cfg(target_arch = "arm")]
pub mod yaw_pitch_roll;

#[cfg(target_arch = "arm")]
pub mod pid_controller;

#[cfg(target_arch = "arm")]
pub mod axis;

pub const MAX_LIFT: f32 = 15f32;

pub const YAW_RATE: f32 = 80f32;
pub const PITCH_DEGREE: f32 = 20f32;
pub const ROLL_DEGREE: f32 = 20f32;
