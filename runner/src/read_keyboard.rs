use std::{
    thread::sleep,
    time::{Duration, Instant},
};

use crossterm::{
    event::{self, Event, KeyCode},
    terminal::disable_raw_mode,
};
use my_hdlc::{
    command::{DeviceCommand, FSMState, WirelessOptions},
    pc_command::ManualInput,
};
use tudelft_serial_upload::serial2::SerialPort;

pub fn send_transition(
    state: my_hdlc::command::FSMState,
    rcv: &mut my_hdlc::HdlcTransceiver,
    cur_mode: &mut FSMState,
    serial: &mut SerialPort,
) {
    //TODO: Only try this transition if its possible
    //In other words, try to perform it in the runner first, and only then send it
    const LATENCY_WAIT_TIME: Duration = Duration::from_millis(30);
    const WAIT_TIME: Duration = Duration::from_micros(100);

    let mut buf = [0u8; my_hdlc::BUFFER_SIZE];
    loop {
        let send_buffer = rcv.write_structure::<DeviceCommand>(&DeviceCommand::ChangeMode(state));

        serial.write(&send_buffer.0[0..send_buffer.1]);

        let mut to_break = false;

        let mut cur_time: Instant = Instant::now();
        loop {
            if cur_time.elapsed() >= LATENCY_WAIT_TIME {
                break;
            }
        }
        //wait for ack
        if let Ok(num) = serial.read(&mut buf[0..rcv.remaining_bytes]) {
            rcv.add_bytes(&buf[0..num]);
        }

        // the number of loop iterations below is chosen at random
        cur_time = Instant::now();

        loop {
            if cur_time.elapsed() >= WAIT_TIME {
                break;
            }
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

pub fn read_keyboard(
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
                    send_transition(my_hdlc::command::FSMState::SafeMode, rcv, cur_mode, serial);
                }
                KeyCode::Char('2') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::ManualMode,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
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
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::YawControl,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
                }
                KeyCode::Char('5') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::FullControlMode,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
                }
                KeyCode::Char('6') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::RawSensorsFullControlMode,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
                }
                KeyCode::Char('7') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::HeightControlMode,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
                }
                KeyCode::Char('8') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::WirelessMode,
                            rcv,
                            cur_mode,
                            serial,
                        );
                    }
                }
                KeyCode::Char('e') => {
                    disable_raw_mode().unwrap();
                }
                KeyCode::Esc => {
                    keyboard_trim.set_panic(true);
                }
                KeyCode::Char(' ') => {
                    keyboard_trim.set_panic(true);
                }
                KeyCode::Char('1') => {
                    keyboard_trim.set_panic(true);
                }
                // Lift trim
                KeyCode::Char('a') => keyboard_trim.increment_lift(2), //throttle up
                KeyCode::Char('z') => keyboard_trim.increment_lift(-2), //throttle down
                //
                // // Roll trim
                KeyCode::Right => keyboard_trim.increment_roll(-2), //roll down  right arrow key
                KeyCode::Left => keyboard_trim.increment_roll(2),   //roll up     left arrow key
                //
                // // Pitch trim
                KeyCode::Up => keyboard_trim.increment_pitch(2), // pitch up  down arrow key
                KeyCode::Down => keyboard_trim.increment_pitch(-2), // pitch down up arrow key
                //
                // // Yaw trim
                KeyCode::Char('q') => keyboard_trim.increment_yaw(-2), //yaw down
                KeyCode::Char('w') => keyboard_trim.increment_yaw(2),  //yaw up
                //
                KeyCode::Char('u') => keyboard_trim.increment_yaw_p_trim(2f32), //yaw up
                KeyCode::Char('j') => keyboard_trim.increment_yaw_p_trim(-2f32), //yaw up
                KeyCode::Char('i') => keyboard_trim.increment_roll_pitch_p_trim(500f32), //yaw up
                KeyCode::Char('k') => keyboard_trim.increment_roll_pitch_p_trim(-500f32), //yaw up
                KeyCode::Char('o') => keyboard_trim.increment_roll_pitch_d_trim(500f32), //yaw up
                KeyCode::Char('l') => keyboard_trim.increment_roll_pitch_d_trim(-500f32), //yaw up
                //TODO: missing the reset of the maps of page
                // https://cese.ewi.tudelft.nl/embedded-systems-lab/resources/interface-requirements.html
                _ => {}
            }
        }
    }
}
