use my_hdlc::pc_command::ManualInput;

use evdev::*;

// defines the max rate values for each aerial maneuver
// defined in degree per seconds

//------------------------------------------------------

// in kg
const DRONE_WEIGHT: f32 = 4.2;

// in N(ewtons)
const HOVER_FORCE: f32 = 9.8 * DRONE_WEIGHT;

// pub const MAX_LIFT: f32 = HOVER_FORCE * 6.0;
pub const MAX_LIFT: f32 = 10000f32;

//------------------------------------------------------

const YAW_RATE: f32 = 2000f32;
const PITCH_RATE: f32 = 2000f32;
const ROLL_RATE: f32 = 2000f32;

const THRESHOLD: f32 = 10f32;
//------------------------------------------------------

pub fn read_joystick(device: &mut Option<Device>, joystick_input: &mut ManualInput) {
    if device.is_some() {
        if let Ok(events) = device.as_mut().unwrap().fetch_events() {
            for event in events {
                match event.destructure() {
                    //trigger button; this should activate panic mode
                    EventSummary::Key(_, key_type, 1) => match key_type {
                        evdev::KeyCode::BTN_TRIGGER => {
                            joystick_input.set_panic(true);
                        }
                        _ => {}
                    },
                    EventSummary::AbsoluteAxis(_, axis, value) => {
                        let mut v = value as f32;
                        if v < THRESHOLD {
                            v = 0f32;
                        }
                        match axis {
                            AbsoluteAxisCode::ABS_THROTTLE => {
                                joystick_input.set_lift((((255.0 - v) / 255.0) * MAX_LIFT) as i32);
                            }
                            AbsoluteAxisCode::ABS_X => {
                                joystick_input
                                    .set_roll((((v / 512.0) as f32 - 1.0) * ROLL_RATE) as i32);
                            }
                            AbsoluteAxisCode::ABS_Y => {
                                joystick_input
                                    .set_pitch((((v / 512.0) as f32 - 1.0) * PITCH_RATE) as i32);
                            }
                            AbsoluteAxisCode::ABS_RZ => {
                                // have to check what the standard value for this axis is
                                joystick_input
                                    .set_yaw((((v / 128.0) as f32 - 1.0) * YAW_RATE) as i32);
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
        }
        // println!("{:?}", joystick_input.clone());
    }
}

pub fn combine_inputs(trim: &ManualInput, joy: &ManualInput) -> ManualInput {
    //Clamp to prevent values going outside range and crashing the drone
    ManualInput::new(
        (trim.get_lift() + joy.get_lift()).clamp(0, MAX_LIFT as i32),
        (trim.get_roll() + joy.get_roll()).clamp(-ROLL_RATE as i32, ROLL_RATE as i32),
        (trim.get_pitch() + joy.get_pitch()).clamp(-PITCH_RATE as i32, PITCH_RATE as i32),
        (trim.get_yaw() + joy.get_yaw()).clamp(-YAW_RATE as i32, YAW_RATE as i32),
    )
}
