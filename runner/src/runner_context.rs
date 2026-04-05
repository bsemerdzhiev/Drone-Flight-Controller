use evdev::Device;
use my_hdlc::HdlcTransceiver;
use std::{os::unix::net::UnixStream, sync::Mutex};
use tudelft_serial_upload::serial2::SerialPort;

pub struct RunnerContext {
    pub rcv_mut: Mutex<HdlcTransceiver>,
    pub device_mut: Mutex<Option<Device>>,
    pub serial_mut: Mutex<SerialPort>,
    pub python_stream_mut: Mutex<UnixStream>,

    pub keyboard_trim_mut: Mutex<ManualInput>,
    pub joystick_input_mut: Mutex<ManualInput>,
    pub joystick_disconnected_mut: Mutex<bool>,
}

impl RunnerContext {
    pub fn with_rcv<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut HdlcTransceiver) -> R,
    {
        f(&mut self.rcv_mut.lock().unwrap())
    }

    pub fn with_device<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut Option<Device>) -> R,
    {
        f(&mut self.device_mut.lock().unwrap())
    }

    pub fn with_serial<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut SerialPort) -> R,
    {
        f(&mut self.serial_mut.lock().unwrap())
    }

    pub fn with_python_stream<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut UnixStream) -> R,
    {
        f(&mut self.python_stream_mut.lock().unwrap())
    }

    pub fn with_keyboard_trim<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut ManualInput) -> R,
    {
        f(&mut self.keyboard_trim_mut.lock().unwrap())
    }

    pub fn with_joystick_input<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut ManualInput) -> R,
    {
        f(&mut self.joystick_input_mut.lock().unwrap())
    }

    pub fn with_joystick_disconnected<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut bool) -> R,
    {
        f(&mut self.joystick_disconnected_mut.lock().unwrap())
    }
}

const THRESHOLD: f32 = 0.03;

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
