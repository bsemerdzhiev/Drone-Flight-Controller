use std::thread::sleep;
use std::time::{Duration, Instant};

use std::io::Write;
use std::os::unix::net::UnixStream;

use crossterm::terminal::enable_raw_mode;
use evdev::{enumerate, AbsoluteAxisCode, Device};
use my_hdlc::command::DeviceCommand;
use my_hdlc::pc_command::{ManualDroneInput, ManualDroneTrims};
use my_hdlc::STUFFED_MESSAGE_SIZE;
use my_hdlc::{command::FSMState, HdlcTransceiver};
use tudelft_serial_upload::serial2::SerialPort;

use std::sync::Arc;
use std::sync::Mutex;

use crate::read_keyboard::send_transition;
use crate::runner_context::RunnerContext;
use crate::{
    read_joystick::{combine_inputs, read_joystick},
    read_keyboard::read_keyboard,
};
use my_hdlc::telemetry_data::TELEMETERY_DATA_SIZE;

const DEBUG_BOARD_MODE: bool = true;

pub fn downlink_main_loop(ctx: &Arc<RunnerContext>) {
    if !DEBUG_BOARD_MODE {
        ctx.with_device(|s| *s = Some(find_flight_stick().expect("Cannot find flight stick")))
    }
    enable_raw_mode().unwrap();

    let mut joystick_turn = true;

    loop {
        let dev_stat = find_flight_stick();

        if dev_stat.is_some() || DEBUG_BOARD_MODE {
            {
                let mut joystick_disconnected = ctx.joystick_disconnected_mut.lock().unwrap();
                let mut device = ctx.device_mut.lock().unwrap();

                if *joystick_disconnected {
                    *device = dev_stat;
                    *joystick_disconnected = false;
                }
            }

            read_joystick(&ctx);

            read_keyboard(&ctx);
        } else {
            println!("Joystick disconnected!\r");
            ctx.with_joystick_disconnected(|s| *s = true);
        }
        check_for_panic(ctx);

        let cmd = combine_inputs(ctx);
        let cmd_for_ui = cmd.clone();

        {
            let mut rcv = ctx.rcv_mut.lock().unwrap();
            let mut serial = ctx.serial_mut.lock().unwrap();
            let wireless_mode: bool = ctx.with_is_wireless(|s| *s);

            let send_buffer = {
                if (joystick_turn) {
                    rcv.write_structure::<DeviceCommand>(&DeviceCommand::ManualInput(
                        ManualDroneInput::from(cmd),
                    ))
                } else {
                    rcv.write_structure::<DeviceCommand>(&DeviceCommand::ManualDroneTrims(
                        ManualDroneTrims::from(cmd),
                    ))
                }
            };
            joystick_turn ^= true;

            println!("Currend mode {}\n\r", wireless_mode);
            if (wireless_mode) {
                ctx.with_wireless_package(|s| {
                    if s.len() == 0 {
                        *s = send_buffer.0[0..send_buffer.1].to_vec();
                    }
                });
            } else {
                serial.write(&send_buffer.0[0..send_buffer.1]);
            }
        }

        let json = serde_json::to_string(&serde_json::json!({
            "ManualInput": {
                "lift": cmd_for_ui.get_lift(),
                "roll": cmd_for_ui.get_roll(),
                "pitch": cmd_for_ui.get_pitch(),
                "yaw": cmd_for_ui.get_yaw(),
                "telemetry_data_size" : STUFFED_MESSAGE_SIZE,

                "yaw_p_trim": cmd_for_ui.yaw_p_trim,
                "roll_pitch_p_trim": cmd_for_ui.roll_pitch_p_trim,
                "roll_pitch_d_trim": cmd_for_ui.roll_pitch_d_trim,
            }
        }))
        .unwrap();

        {
            let mut python_stream = ctx.python_stream_mut.lock().unwrap();

            let _ = python_stream.write_all(json.as_bytes());
            let _ = python_stream.write_all(b"\n");
        }

        sleep(Duration::from_millis(50));
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

fn check_for_panic(ctx: &Arc<RunnerContext>) {
    {
        let mut joy = ctx.joystick_input_mut.lock().unwrap();
        let mut keyboard = ctx.keyboard_trim_mut.lock().unwrap();
        let joystick_disconnected = ctx.joystick_disconnected_mut.lock().unwrap();

        if joy.is_panic_triggered() | keyboard.is_panic_triggered() | *joystick_disconnected {
            send_transition(FSMState::PanicMode, ctx);
            joy.set_panic(false);
            keyboard.set_panic(false);
        }
    }
}
