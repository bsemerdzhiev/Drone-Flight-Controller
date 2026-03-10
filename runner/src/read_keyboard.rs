use std::time::Duration;

use crossterm::event::{self, Event, KeyCode};
use my_hdlc::{
    command::{DeviceCommand, FSMState},
    pc_command::ManualInput,
};
use tudelft_serial_upload::serial2::SerialPort;

pub fn send_transition(
    state: my_hdlc::command::FSMState,
    rcv: &mut my_hdlc::HdlcTransceiver,
    cur_mode: &mut FSMState,
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

        // the number of loop iterations below is chosen at random
        let i_range = if (state == FSMState::PanicMode) {
            10000
        } else {
            100
        };
        for _ in 0..i_range {
            if let Some(x) = rcv.read_structure::<DeviceCommand>() {
                match x {
                    DeviceCommand::Ack => {
                        to_break = true;
                    }
                    _ => {}
                }
            }
        }

        if to_break {
            break;
        }
    }
    *cur_mode = state;
}

pub fn keyboard_trimming(
    keyboard_trim: &mut ManualInput,
    joystick_info: &mut ManualInput,
    rcv: &mut my_hdlc::HdlcTransceiver,
    cur_mode: &mut FSMState,
    serial: &mut SerialPort,
) {
    while event::poll(Duration::from_millis(5)).unwrap() {
        if let Event::Key(key) = event::read().unwrap() {
            match key.code {
                KeyCode::Char('0') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::SafeMode,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
                }
                KeyCode::Char('2') => {
                    send_transition(
                        my_hdlc::command::FSMState::ManualMode,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                KeyCode::Char('3') => {
                    send_transition(
                        my_hdlc::command::FSMState::CalibrationMode,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                KeyCode::Char('4') => {
                    send_transition(
                        my_hdlc::command::FSMState::YawControl,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                KeyCode::Char('5') => {
                    send_transition(
                        my_hdlc::command::FSMState::FullControlMode,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                KeyCode::Char('6') => {
                    send_transition(
                        my_hdlc::command::FSMState::RawSensorsFullControlMode,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                KeyCode::Char('7') => {
                    send_transition(
                        my_hdlc::command::FSMState::HeightControlMode,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                KeyCode::Char('8') => {
                    send_transition(
                        my_hdlc::command::FSMState::WirelessMode,
                        rcv,
                        cur_mode,
                        serial,
                    );
                }
                //TODO: missing the reset of the maps of page
                // https://cese.ewi.tudelft.nl/embedded-systems-lab/resources/interface-requirements.html
                KeyCode::Esc => {
                    keyboard_trim.set_panic(true);
                }
                KeyCode::Char(' ') => {
                    keyboard_trim.set_panic(true);
                }
                KeyCode::Char('1') => {
                    keyboard_trim.set_panic(true);
                }
                _ => {}
            }
        }
    }
}
