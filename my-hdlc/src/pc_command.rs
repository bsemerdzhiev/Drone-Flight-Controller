use core::i32;

use serde::{Deserialize, Serialize};

const MIN_THRESHOLD: i32 = 30;

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Default)]
pub struct ManualInput {
    lift: i32,
    roll: i32,
    pitch: i32,
    yaw: i32,
    enter_panic: bool,
}

impl ManualInput {
    pub fn zero() -> Self {
        Self {
            lift: 0,
            roll: 0,
            pitch: 0,
            yaw: 0,
            enter_panic: false,
        }
    }
    pub fn new(lift: i32, roll: i32, pitch: i32, yaw: i32) -> Self {
        return Self {
            lift,
            roll,
            pitch,
            yaw,
            enter_panic: false,
        };
    }

    pub fn set_lift(&mut self, lift: i32) {
        self.lift = lift;
    }

    pub fn set_roll(&mut self, roll: i32) {
        self.roll = roll;
        if (self.roll.abs() < MIN_THRESHOLD) {
            self.roll = 0;
        }
    }

    pub fn set_pitch(&mut self, pitch: i32) {
        self.pitch = pitch;
        if (self.pitch.abs() < MIN_THRESHOLD) {
            self.pitch = 0;
        }
    }

    pub fn set_yaw(&mut self, yaw: i32) {
        self.yaw = yaw;
        if (self.yaw.abs() < MIN_THRESHOLD) {
            self.yaw = 0;
        }
    }

    pub fn set_panic(&mut self, panic_mode: bool) {
        self.enter_panic = panic_mode
    }

    pub fn get_lift(&self) -> i32 {
        self.lift
    }

    pub fn increment_lift(&mut self, inc: i32) {
        self.lift += inc
    }

    pub fn increment_pitch(&mut self, inc: i32) {
        self.pitch += inc
    }

    pub fn increment_roll(&mut self, inc: i32) {
        self.roll += inc
    }

    pub fn increment_yaw(&mut self, inc: i32) {
        self.yaw += inc
    }

    pub fn get_roll(&self) -> i32 {
        self.roll
    }

    pub fn get_pitch(&self) -> i32 {
        self.pitch
    }

    pub fn get_yaw(&self) -> i32 {
        self.yaw
    }

    pub fn is_panic_triggered(&self) -> bool {
        self.enter_panic
    }

    pub fn is_zeroed(&self) -> bool {
        self.lift == 0 && self.pitch == 0 && self.roll == 0 && self.yaw == 0 && !self.enter_panic
    }
}
