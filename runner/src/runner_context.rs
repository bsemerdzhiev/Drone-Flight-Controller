use evdev::Device;
use my_hdlc::HdlcTransceiver;
use std::{os::unix::net::UnixStream, sync::Mutex};
use tudelft_serial_upload::serial2::SerialPort;

pub struct RunnerContext {
    pub rcv_mut: Mutex<HdlcTransceiver>,
    pub device_mut: Mutex<Option<Device>>,
    pub serial_mut: Mutex<SerialPort>,
    pub python_stream_mut: Mutex<UnixStream>,
    pub manual_input_mut: Mutex<ManualInput>,
}

#[derive(Debug, PartialEq, Clone, Default)]
pub struct ManualInput {
    lift: i16,
    roll: i16,
    pitch: i16,
    yaw: i16,

    pub yaw_p_trim: i16,
    pub roll_pitch_p_trim: i16,
    pub roll_pitch_d_trim: i16,
    enter_panic: bool,
}

impl ManualInput {
    pub fn new(
        lift: i16,
        roll: i16,
        pitch: i16,
        yaw: i16,
        yaw_p_trim: i16,
        roll_pitch_p_trim: i16,
        roll_pitch_d_trim: i16,
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

    pub fn set_lift(&mut self, lift: i16) {
        self.lift = lift;
    }

    pub fn set_roll(&mut self, roll: i16) {
        self.roll = roll;
    }

    pub fn set_pitch(&mut self, pitch: i16) {
        self.pitch = pitch;
    }

    pub fn set_yaw(&mut self, yaw: i16) {
        self.yaw = yaw;
    }

    pub fn set_panic(&mut self, panic_mode: bool) {
        self.enter_panic = panic_mode
    }

    pub fn increment_lift(&mut self, inc: i16) {
        self.set_lift(self.lift + inc);
    }

    pub fn increment_pitch(&mut self, inc: i16) {
        self.set_pitch(self.pitch + inc);
    }

    pub fn increment_roll(&mut self, inc: i16) {
        self.set_roll(self.roll + inc);
    }

    pub fn increment_yaw(&mut self, inc: i16) {
        self.set_yaw(self.yaw + inc);
    }

    pub fn increment_yaw_p_trim(&mut self, inc: i16) {
        self.yaw_p_trim += inc
    }
    pub fn increment_roll_pitch_p_trim(&mut self, inc: i16) {
        self.roll_pitch_p_trim += inc
    }
    pub fn increment_roll_pitch_d_trim(&mut self, inc: i16) {
        self.roll_pitch_d_trim += inc
    }

    pub fn get_lift(&self) -> i16 {
        self.lift
    }

    pub fn get_roll(&self) -> i16 {
        self.roll
    }

    pub fn get_pitch(&self) -> i16 {
        self.pitch
    }

    pub fn get_yaw(&self) -> i16 {
        self.yaw
    }

    pub fn is_panic_triggered(&self) -> bool {
        self.enter_panic
    }

    pub fn is_zeroed(&self) -> bool {
        self.lift.abs() == 0
            && self.pitch.abs() == 0
            && self.roll.abs() == 0
            && self.yaw.abs() == 0
            && !self.enter_panic
    }
}
