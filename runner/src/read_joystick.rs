use evdev::*;
use my_hdlc::pc_command::ManualInput;
use std::sync::Arc;

use crate::runner_context::RunnerContext;

//------------------------------------------------------

const THRESHOLD: f32 = 30f32;
//------------------------------------------------------

pub fn read_joystick(ctx: &Arc<RunnerContext>) {
    let mut joystick_input = ctx.joystick_input_mut.lock().unwrap();
    let mut device = ctx.device_mut.lock().unwrap();

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
                                // joystick_input.set_lift((((255.0 - v) / 255.0) * MAX_LIFT) as i32);
                                joystick_input.set_lift(((255.0 - v) / 255.0));
                            }
                            AbsoluteAxisCode::ABS_X => {
                                joystick_input.set_roll((v / 512.0) - 1.0);
                            }
                            AbsoluteAxisCode::ABS_Y => {
                                joystick_input.set_pitch((v / 512.0) - 1.0);
                            }
                            AbsoluteAxisCode::ABS_RZ => {
                                joystick_input.set_yaw((1.0 - (v / 128.0)));
                            }
                            _ => {}
                        }
                    }
                    _ => {}
                }
            }
        }
    }
}

pub fn combine_inputs(ctx: &Arc<RunnerContext>) -> ManualInput {
    //Clamp to prevent values going outside range and crashing the drone

    let trim = ctx.keyboard_trim_mut.lock().unwrap();
    let joy = ctx.joystick_input_mut.lock().unwrap();
    ManualInput::new(
        (trim.get_lift() + joy.get_lift()).clamp(0.0, 1.0),
        (trim.get_roll() + joy.get_roll()).clamp(-1.0, 1.0),
        (trim.get_pitch() + joy.get_pitch()).clamp(-1.0, 1.0),
        (trim.get_yaw() + joy.get_yaw()).clamp(-1.0, 1.0),
        trim.yaw_p_trim,
        trim.roll_pitch_p_trim,
        trim.roll_pitch_d_trim,
    )
}
