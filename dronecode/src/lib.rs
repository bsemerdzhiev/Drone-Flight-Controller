#![no_std]

#[cfg(test)] //std for testing but not for embedded builds
extern crate std;

extern crate alloc;

pub mod calibration_state;
pub mod full_control_logic;
pub mod yaw_pitch_roll;

#[cfg(target_arch = "arm")]
pub mod states;

#[cfg(target_arch = "arm")]
pub mod util;

#[cfg(target_arch = "arm")]
pub mod filters;
