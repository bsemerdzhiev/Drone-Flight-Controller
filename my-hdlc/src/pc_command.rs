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
pub struct PIDValues {
    pub p_value: f32,
    pub i_value: f32,
    pub d_value: f32,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub enum ManualDroneTrimsEnums {
    Lift(PIDValues),
    Yaw(PIDValues),
    Pitch(PIDValues),
    Roll(PIDValues),
}

impl Default for ManualDroneTrimsEnums {
    fn default() -> Self {
        ManualDroneTrimsEnums::Lift(PIDValues::default())
    }
}

impl From<ManualInput> for ManualDroneInput {
    fn from(input: ManualInput) -> Self {
        Self {
            lift: (input.lift * i16::MAX as f32) as i16,
            roll: (input.roll * i16::MAX as f32) as i16,
            pitch: (input.pitch * i16::MAX as f32) as i16,
            yaw: (input.yaw * i16::MAX as f32) as i16,
        }
    }
}

// impl From<ManualInput> for ManualDroneTrims {
//     fn from(input: ManualInput) -> Self {
//         Self {
//             yaw_p_trim: (input.yaw_p_trim * i16::MAX as f32) as i16,
//             roll_pitch_p_trim: (input.roll_pitch_p_trim * i16::MAX as f32) as i16,
//             roll_pitch_d_trim: (input.roll_pitch_d_trim * i16::MAX as f32) as i16,
//         }
//     }
// }

#[derive(Debug, PartialEq, Clone, Default)]
pub struct ManualInput {
    lift: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,

    pub yaw_p_trim: f32,
    pub roll_pitch_p_trim: f32,
    pub roll_pitch_d_trim: f32,
    enter_panic: bool,
}

impl ManualInput {
    pub fn new(
        lift: f32,
        roll: f32,
        pitch: f32,
        yaw: f32,
        yaw_p_trim: f32,
        roll_pitch_p_trim: f32,
        roll_pitch_d_trim: f32,
    ) -> Self {
        return Self {
            lift,
            roll,
            pitch,
            yaw,

            yaw_p_trim: yaw_p_trim,
            roll_pitch_p_trim: roll_pitch_p_trim,
            roll_pitch_d_trim: roll_pitch_d_trim,
            enter_panic: false,
        };
    }

    pub fn set_lift(&mut self, lift: f32) {
        self.lift = lift;
    }

    pub fn set_roll(&mut self, roll: f32) {
        self.roll = roll;
    }

    pub fn set_pitch(&mut self, pitch: f32) {
        self.pitch = pitch;
    }

    pub fn set_yaw(&mut self, yaw: f32) {
        self.yaw = yaw;
    }

    pub fn set_panic(&mut self, panic_mode: bool) {
        self.enter_panic = panic_mode
    }

    pub fn increment_lift(&mut self, inc: f32) {
        self.set_lift(self.lift + inc);
    }

    pub fn increment_pitch(&mut self, inc: f32) {
        self.set_pitch(self.pitch + inc);
    }

    pub fn increment_roll(&mut self, inc: f32) {
        self.set_roll(self.roll + inc);
    }

    pub fn increment_yaw(&mut self, inc: f32) {
        self.set_yaw(self.yaw + inc);
    }

    pub fn increment_yaw_p_trim(&mut self, inc: f32) {
        self.yaw_p_trim += inc
    }
    pub fn increment_roll_pitch_p_trim(&mut self, inc: f32) {
        self.roll_pitch_p_trim += inc
    }
    pub fn increment_roll_pitch_d_trim(&mut self, inc: f32) {
        self.roll_pitch_d_trim += inc
    }

    pub fn get_lift(&self) -> f32 {
        self.lift
    }

    pub fn get_roll(&self) -> f32 {
        self.roll
    }

    pub fn get_pitch(&self) -> f32 {
        self.pitch
    }

    pub fn get_yaw(&self) -> f32 {
        self.yaw
    }

    pub fn is_panic_triggered(&self) -> bool {
        self.enter_panic
    }

    pub fn is_zeroed(&self) -> bool {
        self.lift.abs() < THRESHOLD
            && self.pitch.abs() < THRESHOLD
            && self.roll.abs() < THRESHOLD
            && self.yaw.abs() < THRESHOLD
            && !self.enter_panic
    }
}
