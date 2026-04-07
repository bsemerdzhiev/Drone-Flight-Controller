use evdev::Device;
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};
use std::{collections::VecDeque, os::unix::net::UnixStream, sync::Mutex};
use tudelft_serial_upload::serial2::SerialPort;

pub struct RunnerContext {
    pub rcv_mut: Mutex<HdlcTransceiver>,
    pub device_mut: Mutex<Option<Device>>,
    pub serial_mut: Mutex<SerialPort>,
    pub python_stream_mut: Mutex<UnixStream>,

    pub keyboard_trim_mut: Mutex<ManualInput>,
    pub joystick_input_mut: Mutex<ManualInput>,
    pub joystick_disconnected_mut: Mutex<bool>,

    pub is_wireless_mut: Mutex<bool>,
    pub package_sender_mut: Mutex<VecDeque<Vec<u8>>>,

    pub current_state: Mutex<FSMState>,
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

    pub fn with_is_wireless<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut bool) -> R,
    {
        f(&mut self.is_wireless_mut.lock().unwrap())
    }

    pub fn with_package_sender<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut VecDeque<Vec<u8>>) -> R,
    {
        f(&mut self.package_sender_mut.lock().unwrap())
    }

    pub fn with_current_state<F, R>(&self, f: F) -> R
    where
        F: FnOnce(&mut FSMState) -> R,
    {
        f(&mut self.current_state.lock().unwrap())
    }
}
