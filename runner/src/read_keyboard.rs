use std::time::Duration;

use crossterm::event::{self, Event, KeyCode};
use my_hdlc::{
    command::{DeviceCommand, FSMState},
    pc_command::ManualInput,
};
use tudelft_serial_upload::serial2::SerialPort;

fn send_transition(
    state: my_hdlc::command::FSMState,
    rcv: &mut my_hdlc::HdlcTransceiver,
    serial: &mut SerialPort,
) {
    let mut buf = [0u8; my_hdlc::BUFFER_SIZE];
    loop {
        let send_buffer = rcv.write_structure::<DeviceCommand>(&DeviceCommand::ChangeMode(state));

        serial.write(&send_buffer.0[0..send_buffer.1]);

        let mut to_break = false;

        //wait for ack
        if let Ok(num) = serial.read(&mut buf[0..rcv.remaining_bytes]) {
            rcv.add_bytes(&buf[0..num]);
        }

        while !rcv.fifo_is_empty() {
            if let Some(x) = rcv.read_structure::<my_hdlc::command::DeviceCommand>() {
                match x {
                    my_hdlc::command::DeviceCommand::Ack => {
                        to_break = true;
                    }
                    _ => {}
                }
                println!("{:?}\n", x);
            }
        }

        if to_break {
            break;
        }
    }
}

pub fn keyboard_trimming(
    keyboard_trim: &mut ManualInput,
    rcv: &mut my_hdlc::HdlcTransceiver,
    serial: &mut SerialPort,
) {
    while event::poll(Duration::from_millis(20)).unwrap() {
        if let Event::Key(key) = event::read().unwrap() {
            match key.code {
                KeyCode::Char('0') => {
                    send_transition(my_hdlc::command::FSMState::SafeMode, rcv, serial);
                }
                KeyCode::Char('1') => {
                    send_transition(my_hdlc::command::FSMState::PanicMode, rcv, serial);
                }
                KeyCode::Char('2') => {
                    send_transition(my_hdlc::command::FSMState::ManualMode, rcv, serial);
                }
                KeyCode::Char('3') => {
                    send_transition(my_hdlc::command::FSMState::CalibrationMode, rcv, serial);
                }
                _ => {}
            }
        }
    }
}
