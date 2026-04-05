use std::{
    sync::Mutex,
    thread::sleep,
    time::{Duration, Instant},
};

use crossterm::{
    event::{self, Event, KeyCode},
    terminal::disable_raw_mode,
};
use my_hdlc::{
    command::{DeviceCommand, FSMState},
    HdlcTransceiver,
};
use std::sync::Arc;
use tudelft_serial_upload::serial2::SerialPort;

use crate::runner_context::{ManualInput, RunnerContext};

pub fn send_transition(
    state: my_hdlc::command::FSMState,
    rcv_mut: &Arc<Mutex<HdlcTransceiver>>,
    serial_mut: &Arc<Mutex<SerialPort>>,
) {
    const WAIT_TIME: Duration = Duration::from_millis(1000);

    let mut buf = Box::new([0u8; my_hdlc::BUFFER_SIZE]);

    {
        let mut rcv = rcv_mut.lock().unwrap();
        let mut serial = serial_mut.lock().unwrap();
        // loop {
        let send_buffer = rcv.write_structure::<DeviceCommand>(&DeviceCommand::ChangeMode(state));

        serial.write(&send_buffer.0[0..send_buffer.1]);

        //NOTE: The section below makes sure that the drone transitions states
        //comment it out if there are issues with the transitions
        // -------------------------------------------------------------------------------------------------
        // let mut to_break = false;
        //
        // let mut cur_time: Instant = Instant::now();
        //
        // // the number of loop iterations below is chosen at random
        // cur_time = Instant::now();
        //
        // loop {
        //     if cur_time.elapsed() >= WAIT_TIME {
        //         break;
        //     }
        //     if let Ok(num) = serial.read(&mut buf[0..rcv.remaining_bytes]) {
        //         rcv.add_bytes(&buf[0..num]);
        //     }
        //
        //     if let Some(x) = rcv.read_structure::<DeviceCommand>() {
        //         match x {
        //             DeviceCommand::Ack => {
        //                 println!("Received ACK for mode transition to {:?}", state);
        //                 return;
        //             }
        //             _ => {}
        //         }
        //     }
        // }
        // -------------------------------------------------------------------------------------------------
    }
    // }
}

pub fn read_keyboard(ctx: &Arc<RunnerContext>) {
    while event::poll(Duration::from_millis(5)).unwrap() {
        if let Event::Key(key) = event::read().unwrap() {
            println!("Keyboard event: {:?}", key.code);
            match key.code {
                KeyCode::Char('0') => {
                    send_transition(my_hdlc::command::FSMState::SafeMode, rcv_mut, serial_mut);
                }
                KeyCode::Char('2') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::ManualMode,
                            rcv_mut,
                            serial_mut,
                        );
                    } else {
                        println!("Ignored ManualMode request because joystick input is not zeroed");
                    }
                }
                KeyCode::Char('3') => {
                    send_transition(
                        my_hdlc::command::FSMState::CalibrationMode,
                        rcv_mut,
                        serial_mut,
                    );
                }
                KeyCode::Char('4') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::YawControl,
                            rcv_mut,
                            serial_mut,
                        );
                    } else {
                        println!("Ignored YawControl request because joystick input is not zeroed");
                    }
                }
                KeyCode::Char('5') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::FullControlMode,
                            rcv_mut,
                            serial_mut,
                        );
                    } else {
                        println!(
                            "Ignored FullControlMode request because joystick input is not zeroed"
                        );
                    }
                }
                KeyCode::Char('6') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::RawSensorsFullControlMode,
                            rcv_mut,
                            serial_mut,
                        );
                    } else {
                        println!("Ignored RawSensorsFullControlMode request because joystick input is not zeroed");
                    }
                }
                KeyCode::Char('7') => {
                    send_transition(
                        my_hdlc::command::FSMState::HeightControlMode,
                        rcv_mut,
                        serial_mut,
                    );
                }
                KeyCode::Char('8') => {
                    if joystick_info.is_zeroed() {
                        send_transition(
                            my_hdlc::command::FSMState::WirelessMode,
                            rcv_mut,
                            serial_mut,
                        );
                    } else {
                        println!(
                            "Ignored WirelessMode request because joystick input is not zeroed"
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
                KeyCode::Char('a') => keyboard_trim.increment_lift(0.1), //throttle up
                KeyCode::Char('z') => keyboard_trim.increment_lift(-0.1), //throttle down
                //
                // // Roll trim
                KeyCode::Right => keyboard_trim.increment_roll(-0.1), //roll down  right arrow key
                KeyCode::Left => keyboard_trim.increment_roll(0.1),   //roll up     left arrow key
                //
                // // Pitch trim
                KeyCode::Up => keyboard_trim.increment_pitch(0.1), // pitch up  down arrow key
                KeyCode::Down => keyboard_trim.increment_pitch(-0.1), // pitch down up arrow key
                //
                // // Yaw trim
                KeyCode::Char('q') => keyboard_trim.increment_yaw(-0.1), //yaw down
                KeyCode::Char('w') => keyboard_trim.increment_yaw(0.1),  //yaw up
                //
                KeyCode::Char('u') => keyboard_trim.increment_yaw_p_trim(0.01f32), //yaw up
                KeyCode::Char('j') => keyboard_trim.increment_yaw_p_trim(-0.01f32), //yaw up
                KeyCode::Char('i') => keyboard_trim.increment_roll_pitch_p_trim(0.005f32), //yaw up
                KeyCode::Char('k') => keyboard_trim.increment_roll_pitch_p_trim(-0.005f32), //yaw up
                KeyCode::Char('o') => keyboard_trim.increment_roll_pitch_d_trim(0.0001f32), //yaw up
                KeyCode::Char('l') => keyboard_trim.increment_roll_pitch_d_trim(-0.0001f32), //yaw up
                //TODO: missing the reset of the maps of page
                // https://cese.ewi.tudelft.nl/embedded-systems-lab/resources/interface-requirements.html
                _ => {}
            }
        }
    }
}
