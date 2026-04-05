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

use crate::runner_context::RunnerContext;

pub fn send_transition(state: my_hdlc::command::FSMState, ctx: &Arc<RunnerContext>) {
    const WAIT_TIME: Duration = Duration::from_millis(1000);

    let can_transition = (ctx.with_current_state(|x| match state {
        FSMState::WirelessMode => match x {
            FSMState::SafeMode => true,
            _ => false,
        },
        _ => true,
    }));

    // ctx.with_is_wireless(|s| *s ^= true);

    let mut buf = Box::new([0u8; my_hdlc::BUFFER_SIZE]);

    {
        let mut rcv = ctx.rcv_mut.lock().unwrap();
        let mut serial = ctx.serial_mut.lock().unwrap();
        // loop {
        let send_buffer = rcv.write_structure::<DeviceCommand>(&DeviceCommand::ChangeMode(state));

        let wireless_mode: bool = ctx.with_is_wireless(|s| *s);

        if (wireless_mode) {
            ctx.with_wireless_package(|s| *s = send_buffer.0[0..send_buffer.1].to_vec());
        } else {
            serial.write(&send_buffer.0[0..send_buffer.1]);
        }

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
            let mut joystick_is_zeroed = false;

            let mut keyboard_trim = ctx.keyboard_trim_mut.lock().unwrap();

            ctx.with_joystick_input(|s| joystick_is_zeroed = s.is_zeroed());

            match key.code {
                KeyCode::Char('0') => {
                    send_transition(my_hdlc::command::FSMState::SafeMode, ctx);
                }
                KeyCode::Char('2') => {
                    if joystick_is_zeroed {
                        send_transition(my_hdlc::command::FSMState::ManualMode, ctx);
                    } else {
                        println!("Ignored ManualMode request because joystick input is not zeroed");
                    }
                }
                KeyCode::Char('3') => {
                    send_transition(my_hdlc::command::FSMState::CalibrationMode, ctx);
                }
                KeyCode::Char('4') => {
                    if joystick_is_zeroed {
                        send_transition(my_hdlc::command::FSMState::YawControl, ctx);
                    } else {
                        println!("Ignored YawControl request because joystick input is not zeroed");
                    }
                }
                KeyCode::Char('5') => {
                    if joystick_is_zeroed {
                        send_transition(my_hdlc::command::FSMState::FullControlMode, ctx);
                    } else {
                        println!(
                            "Ignored FullControlMode request because joystick input is not zeroed"
                        );
                    }
                }
                KeyCode::Char('6') => {
                    if joystick_is_zeroed {
                        send_transition(my_hdlc::command::FSMState::RawSensorsFullControlMode, ctx);
                    } else {
                        println!("Ignored RawSensorsFullControlMode request because joystick input is not zeroed");
                    }
                }
                KeyCode::Char('7') => {
                    send_transition(my_hdlc::command::FSMState::HeightControlMode, ctx);
                }
                KeyCode::Char('8') => {
                    if joystick_is_zeroed {
                        send_transition(my_hdlc::command::FSMState::WirelessMode, ctx);
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
