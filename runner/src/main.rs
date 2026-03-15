use crate::read_joystick::combine_inputs;
use crate::read_joystick::read_joystick;
use crate::read_keyboard::read_keyboard;
use crate::read_keyboard::send_transition;

use crossterm::terminal::disable_raw_mode;
use crossterm::terminal::enable_raw_mode;
use my_hdlc::command::DeviceCommand;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use rand;

use crossterm::event::{self, Event, KeyCode};
use evdev::{enumerate, AbsoluteAxisCode, Device};
pub use my_hdlc::pc_command;
use rand::RngExt;
use std::env::args;

use std::path::PathBuf;
use std::process::exit;
use std::process::Command;
use std::time::Duration;
use std::time::Instant;
use tudelft_serial_upload::serial2::SerialPort;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

pub use my_hdlc::HdlcTransceiver;
use my_hdlc::STUFFED_MESSAGE_SIZE;

mod read_joystick;
mod read_keyboard;

const PRINT_DRONE_DATA: bool = true;
const DEBUG_BOARD_MODE: bool = false;
fn main() {
    // get a filename from the command line. This filename will be uploaded to the drone
    // note that if no filename is given, the upload to the drone does not fail.
    // `upload_file_or_stop` will still try to detect the serial port on which the drone
    // is attached. This may be useful if you don't want to actually change the code on the
    // drone, but you do want to rerun your UI. In that case you simply don't provide any
    // command line parameter.
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);

    // The code below shows a very simple start to a PC-side receiver of data from the drone.
    // You can extend this into an entire interface to the drone written in Rust. However,
    // if you are more comfortable writing such an interface in any other programming language
    // you like (for example, python isn't a bad choice), you can also call that here. The
    // commented function below gives an example of how to do that with python, but again
    // you don't need to choose python.

    start_interface(&port);

    // open the serial port we got back from `upload_file_or_stop`. This is the same port
    // as the upload occurred on, so we know that we can communicate with the drone over
    // this port.
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_millis(400)).unwrap();

    let mut keyboard_trim = ManualInput::zero();
    let mut joystick_input = ManualInput::zero();

    // let mut device = find_flight_stick().expect("Cannot find flight stick"); //comment this when testing without stick
    let mut device: Option<Device> = None;

    if !DEBUG_BOARD_MODE {
        device = Some(find_flight_stick().expect("Cannot find flight stick"));
    }

    // for timing and sending inputs at fixed rate
    let send_period = Duration::from_millis(40);
    let mut last_send = Instant::now();

    let mut buf = [0u8; my_hdlc::BUFFER_SIZE];

    let mut rcv = my_hdlc::HdlcTransceiver::new();

    let mut rng = rand::rng();
    // infinitely print whatever the drone sends us
    let mut rcv: HdlcTransceiver = HdlcTransceiver::new();

    // infinitely print whatever the drone sends us

    enable_raw_mode().unwrap();
    let mut iterations_without_message = 0u16;
    let mut current_mode = FSMState::SafeMode;
    let mut joystick_disconnected = false;
    loop {
        let dev_stat = find_flight_stick();
        if dev_stat.is_some() || DEBUG_BOARD_MODE {
            if joystick_disconnected {
                device = dev_stat;
                joystick_disconnected = false;
            }
            read_joystick(&mut device, &mut joystick_input);
            // println!("{}", joystick_input);
            read_keyboard(
                &mut keyboard_trim,
                &mut joystick_input,
                &mut rcv,
                &mut current_mode,
                &mut serial,
            );
        } else {
            println!("Joystick disconnected!");
            joystick_disconnected = true;
        }
        check_for_panic(
            &mut joystick_input,
            &mut keyboard_trim,
            &mut current_mode,
            &mut joystick_disconnected,
            &mut rcv,
            &mut serial,
        );
        if last_send.elapsed() >= send_period {
            let cmd = combine_inputs(&keyboard_trim, &joystick_input);

            let send_buffer = rcv.write_structure::<my_hdlc::command::DeviceCommand>(
                &my_hdlc::command::DeviceCommand::ManualInput(cmd),
            );

            serial.write(&send_buffer.0[0..send_buffer.1]);
            last_send += send_period;
        }

        if let Ok(num) = serial.read(&mut buf[0..rcv.remaining_bytes]) {
            rcv.add_bytes(&buf[0..num]);
        }
        if let Some(x) = rcv.read_structure::<my_hdlc::command::DeviceCommand>() {
            iterations_without_message = 0;
            if PRINT_DRONE_DATA {
                println!("{:?}\r", x);
            }
        } else {
            iterations_without_message += 1;
        }
        if iterations_without_message == 200u16 {
            println!("Board Disconnected!!");
            break;
        }
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
    current_mode: &mut FSMState,
    joystick_disconnected: &mut bool,
    rcv: &mut HdlcTransceiver,
    serial: &mut SerialPort,
) {
    if joy.is_panic_triggered() | keyboard.is_panic_triggered() | *joystick_disconnected {
        let send_buffer = rcv.write_structure::<my_hdlc::command::DeviceCommand>(
            &my_hdlc::command::DeviceCommand::ChangeMode(my_hdlc::command::FSMState::PanicMode),
        );
        send_transition(FSMState::PanicMode, rcv, current_mode, serial);
        joy.set_panic(false);
        keyboard.set_panic(false);
    }
}

#[allow(unused)]
fn start_interface(port: &PathBuf) {
    let mut cmd = Command::new("python");
    cmd
        // there must be a `my_interface.py` file of course
        .arg("my_interface.py")
        // pass the serial port as a command line parameter to the python program
        .arg(port.to_str().unwrap());

    match cmd.output() {
        Err(e) => {
            eprintln!("{}", e);
            exit(1);
        }
        Ok(i) if !i.status.success() => exit(i.status.code().unwrap_or(1)),
        Ok(_) => {}
    }
}
