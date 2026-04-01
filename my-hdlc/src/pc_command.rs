use core::f32;
use core::fmt;
use core::i32;

use serde::{Deserialize, Serialize};

const THRESHOLD: f32 = 0.03;

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Default)]
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
    pub fn zero() -> Self {
        Self {
            lift: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,

            yaw_p_trim: 0f32,
            roll_pitch_p_trim: 0f32,
            roll_pitch_d_trim: 0f32,
            enter_panic: false,
        }
    }
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

impl fmt::Display for ManualInput {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // write!(
        //     f,
        //     "x: {}, y: {}, steps: {}",
        //     (self.pos_x as i16 - self.starting_x as i16),
        //     (self.pos_y as i16 - self.starting_y as i16),
        //     self.step_count
        // )
        write!(
            f,
            "Pitch: {}, Roll: {}, Yaw: {}, Lift: {}, Enter Panic? :{}",
            self.pitch, self.roll, self.yaw, self.lift, self.enter_panic
        )
    }
}
