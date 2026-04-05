use core::f32;
use core::fmt;
use core::i32;

use serde::{Deserialize, Serialize};

const THRESHOLD: f32 = 0.03;

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Default)]
pub struct ManualDroneInput {
    pub lift: i16,
    pub roll: i16,
    pub pitch: i16,
    pub yaw: i16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Default)]
pub struct ManualDroneTrims {
    pub yaw_p_trim: i16,
    pub roll_pitch_p_trim: i16,
    pub roll_pitch_d_trim: i16,
}
