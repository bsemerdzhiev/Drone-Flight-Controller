use std::thread::sleep;
use std::time::{Duration, Instant};

use std::io::Write;
use std::os::unix::net::UnixStream;

use crossterm::terminal::enable_raw_mode;
use evdev::{enumerate, AbsoluteAxisCode, Device};
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};
use tudelft_serial_upload::serial2::SerialPort;

use std::sync::Arc;
use std::sync::Mutex;

use crate::read_keyboard::send_transition;
use crate::{
    read_joystick::{combine_inputs, read_joystick},
    read_keyboard::read_keyboard,
};

const DEBUG_BOARD_MODE: bool = true;

pub fn downlink_main_loop(
    rcv_mut: &Arc<Mutex<HdlcTransceiver>>,
    serial_mut: &Arc<Mutex<SerialPort>>,
    python_stream_mut: &Arc<Mutex<UnixStream>>,
) {
    let mut device: Option<Device> = None;

    let mut keyboard_trim = ManualInput::zero();
    let mut joystick_input = ManualInput::zero();

    let mut joystick_disconnected = false;

    if !DEBUG_BOARD_MODE {
        device = Some(find_flight_stick().expect("Cannot find flight stick"));
    }
    enable_raw_mode().unwrap();
    loop {
        let dev_stat = find_flight_stick();

        if dev_stat.is_some() || DEBUG_BOARD_MODE {
            if joystick_disconnected {
                device = dev_stat;
                joystick_disconnected = false;
            }
            read_joystick(&mut device, &mut joystick_input);

            read_keyboard(&mut keyboard_trim, &mut joystick_input, rcv_mut, serial_mut);
        } else {
            println!("Joystick disconnected!\r");
            joystick_disconnected = true;
        }
        check_for_panic(
            &mut joystick_input,
            &mut keyboard_trim,
            &mut joystick_disconnected,
            rcv_mut,
            serial_mut,
        );

        let cmd = combine_inputs(&keyboard_trim, &joystick_input);
        let cmd_for_ui = cmd.clone();

        {
            let mut rcv = rcv_mut.lock().unwrap();
            let mut serial = serial_mut.lock().unwrap();

            let send_buffer = rcv.write_structure::<my_hdlc::command::DeviceCommand>(
                &my_hdlc::command::DeviceCommand::ManualInput(cmd),
            );

            serial.write(&send_buffer.0[0..send_buffer.1]);
        }

        let json = serde_json::to_string(&serde_json::json!({
            "ManualInput": {
                "lift": cmd_for_ui.get_lift(),
                "roll": cmd_for_ui.get_roll(),
                "pitch": cmd_for_ui.get_pitch(),
                "yaw": cmd_for_ui.get_yaw(),
            }
        }))
        .unwrap();

        {
            let mut python_stream = python_stream_mut.lock().unwrap();

            let _ = python_stream.write_all(json.as_bytes());
            let _ = python_stream.write_all(b"\n");
        }

        sleep(Duration::from_millis(200));
    }
}

fn find_flight_stick() -> Option<Device> {
    for (path, _) in enumerate() {
        if let Ok(dev) = Device::open(&path) {
            let name = dev.name().unwrap_or("Unknown");
            if name.contains("Logitech") {
                dev.set_nonblocking(true)
                    .expect("Failed to set joystick to nonblocking");
                return Some(dev);
            }
        }
    }
    None
}

fn check_for_panic(
    joy: &mut ManualInput,
    keyboard: &mut ManualInput,
    joystick_disconnected: &mut bool,
    rcv_mut: &Arc<Mutex<HdlcTransceiver>>,
    serial: &Arc<Mutex<SerialPort>>,
) {
    if joy.is_panic_triggered() | keyboard.is_panic_triggered() | *joystick_disconnected {
        send_transition(FSMState::PanicMode, rcv_mut, serial);
        joy.set_panic(false);
        keyboard.set_panic(false);
    }
}
