use my_hdlc::pc_command::ManualInput;

use evdev::*;

// defines the max rate values for each aerial maneuver
// defined in degree per seconds

const YAW_RATE: i16 = 200;
const PITCH_RATE: i16 = 200;
const ROLL_RATE: i16 = 200;

//------------------------------------------------------

pub fn read_joystick(device: &mut Device, joystick_input: &mut ManualInput) {
    if let Ok(events) = device.fetch_events() {
        for event in events {
            match event.destructure() {
                //trigger button; this should activate panic mode
                EventSummary::Key(_, key_type, 1) => match key_type {
                    evdev::KeyCode::BTN_TRIGGER => {
                        todo!()
                    }
                    _ => {}
                },
                EventSummary::AbsoluteAxis(_, axis, value) => {
                    let v = value as f32;
                    match axis {
                        AbsoluteAxisCode::ABS_THROTTLE => {
                            joystick_input.set_lift(v / 255.0);
                        }
                        AbsoluteAxisCode::ABS_X => {
                            joystick_input.set_roll((v - 128.0) / 128.0);
                        }
                        AbsoluteAxisCode::ABS_Y => {
                            joystick_input.set_pitch(-(v - 128.0) / 128.0);
                        }
                        AbsoluteAxisCode::ABS_RY => {
                            // have to check what the standard value for this axis is
                            joystick_input.set_yaw((v - 128.0) / 128.0);
                        }
                        _ => {}
                    }
                }
                _ => {}
            }
        }
    }
}

pub fn combine_inputs(trim: &ManualInput, joy: &ManualInput) -> ManualInput {
    //Clamp to prevent values going outside range and crashing the drone
    ManualInput::new(
        (trim.get_lift() + joy.get_lift()).clamp(0.0, 1.0),
        (trim.get_roll() + joy.get_roll()).clamp(-1.0, 1.0),
        (trim.get_pitch() + joy.get_pitch()).clamp(-1.0, 1.0),
        (trim.get_yaw() + joy.get_yaw()).clamp(-1.0, 1.0),
    )
}
