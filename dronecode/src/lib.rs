#![no_std]



#[cfg(test)] //std for testing but not for embedded builds
extern crate std;

pub mod calibration_state;
pub mod full_control_logic;

#[cfg(target_arch = "arm")]
pub mod states;


