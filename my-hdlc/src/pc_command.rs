use serde::{Deserialize, Serialize};

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
    }

    pub fn set_pitch(&mut self, pitch: i32) {
        self.pitch = pitch;
    }

    pub fn set_yaw(&mut self, yaw: i32) {
        self.yaw = yaw;
    }

    pub fn set_panic(&mut self, panic_mode: bool) {
        self.enter_panic = panic_mode
    }

    pub fn get_lift(&self) -> i32 {
        self.lift
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
}
