use std::env::args;
use std::path::PathBuf;
use std::process::{exit, Command};
use std::time::Duration;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use tudelft_serial_upload::serial2::SerialPort;
use crossterm::event::{self, Event, KeyCode};
use evdev::{Device, enumerate, AbsoluteAxisCode};

struct ManualInput {
    lift: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
}

impl ManualInput {
    fn zero() -> Self {
        Self {
            lift: 0.0,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
        }
    }
}

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

    // start_interface(&port);

    // open the serial port we got back from `upload_file_or_stop`. This is the same port
    // as the upload occurred on, so we know that we can communicate with the drone over
    // this port.
    let mut serial = SerialPort::open(port, 115200).unwrap();
    serial.set_read_timeout(Duration::from_secs(1)).unwrap();


    let mut keyboard_trim = ManualInput::zero();
    let mut joystick_input = ManualInput::zero();
    let mut device = find_flight_stick().expect("Cannot find flight stick"); //comment this when testing without stick

    // for timing and sending inputs at fixed rate
    let send_period = Duration::from_millis(20);
    let mut last_send = Instant::now();

    
    let mut buf = [0u8; 255];
    crossterm::terminal::enable_raw_mode().unwrap(); // This is for non-blocking reading inputs from keyboard
    loop {
        // ----------------------------------------------
        // (1) Read joystick input
        // ----------------------------------------------
        read_joystick(&mut device, &mut joystick_input);
        // ----------------------------------------------
        // (2) Read keyboard input 
        // ----------------------------------------------
        keyboard_trimming(&mut keyboard_trim);

        // ----------------------------------------------
        // (3) Combine inputs and send at fixed rate
        // ----------------------------------------------
        if last_send.elapsed() >= send_period {
            last_send = Instant::now();

            let cmd = combine_inputs(&keyboard_trim, &joystick_input);

            println!(
                "L {:.2} R {:.2} P {:.2} Y {:.2}",
                cmd.lift, cmd.roll, cmd.pitch, cmd.yaw
            );

            // Later:
            // serial.write_all(&encode_command(cmd)).unwrap();
        }
        
        // ----------------------------------------------
        // (4) Read from drone
        // ----------------------------------------------
        // infinitely print whatever the drone sends us
        if let Ok(num) = serial.read(&mut buf) {
            print!("{}", String::from_utf8_lossy(&buf[0..num]));
        }
    }
    crossterm::terminal::disable_raw_mode().unwrap(); //Stop terminal behave strangely
}

fn read_joystick(device: &mut Device, joystick_input: &mut ManualInput) {
    use evdev::*;
    if let Ok(events) = device.fetch_events() {
        for event in events {
            match event.destructure(){
                    //trigger button; this should activate panic mode
                    EventSummary::Key(_, key_type, 1) => {
                        match key_type {
                            evdev::KeyCode::BTN_TRIGGER => {
                                todo!()
                            },
                            _ => {}
                        }
                    },
                    EventSummary::AbsoluteAxis(_, axis, value) => {
                        let v = value as f32;
                        match axis {
                            AbsoluteAxisCode::ABS_THROTTLE => {
                                joystick_input.lift = 1.0 - (v / 255.0);
                            },
                            AbsoluteAxisCode::ABS_X => {
                                joystick_input.roll = (v - 128.0) / 128.0;
                            },
                            AbsoluteAxisCode::ABS_Y => {
                                joystick_input.pitch = -(v - 128.0) / 128.0;
                            },
                            AbsoluteAxisCode::ABS_RY => {
                                // have to check what the standard value for this axis is
                                joystick_input.yaw = (v - 128.0) / 128.0;
                            },
                            _ => {}
                        }
                    },
                    _ => {}
                }
        }
    }
}

fn keyboard_trimming(keyboard_trim: &mut ManualInput) {
    while event::poll(Duration::from_millis(0)).unwrap() {
        if let Event::Key(key) = event::read().unwrap() {
            match key.code {
                // Lift trim
                KeyCode::Char('a') => keyboard_trim.lift += 0.01, //throttle up
                KeyCode::Char('z') => keyboard_trim.lift -= 0.01, //throttle down

                // Roll trim
                KeyCode::Right => keyboard_trim.roll -= 0.02, //roll down  right arrow key
                KeyCode::Left => keyboard_trim.roll += 0.02, //roll up     left arrow key

                // Pitch trim
                KeyCode::Char('i') => keyboard_trim.pitch += 0.02, // pitch up  down arrow key
                KeyCode::Char('k') => keyboard_trim.pitch -= 0.02, // pitch down up arrow key

                // Yaw trim
                KeyCode::Char('q') => keyboard_trim.yaw -= 0.02, //yaw down
                KeyCode::Char('w') => keyboard_trim.yaw += 0.02, //yaw up

                _ => {}
            }
        }
    }

    // Clamp trim for safety
    keyboard_trim.lift = keyboard_trim.lift.clamp(0.0, 1.0);
    keyboard_trim.roll = keyboard_trim.roll.clamp(-0.5, 0.5);
    keyboard_trim.pitch = keyboard_trim.pitch.clamp(-0.5, 0.5);
    keyboard_trim.yaw = keyboard_trim.yaw.clamp(-0.5, 0.5);
}

fn combine_inputs(
    trim: &ManualInput,
    joy: &ManualInput,
) -> ManualInput {
    //Clamp to prevent values going outside range and crashing the drone
    ManualInput {
        lift: (trim.lift + joy.lift).clamp(0.0, 1.0),
        roll: (trim.roll + joy.roll).clamp(-1.0, 1.0),
        pitch: (trim.pitch + joy.pitch).clamp(-1.0, 1.0),
        yaw: (trim.yaw + joy.yaw).clamp(-1.0, 1.0),
    }
}

fn find_flight_stick() -> Option<Device> {
    for (path, _) in enumerate() {
        if let Ok(dev) = Device::open(&path) {
            let name = dev.name().unwrap_or("Unknown");
            if name.contains("Logitech") {
                return Some(dev);
            }
        }
    }
    None
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
