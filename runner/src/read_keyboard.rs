use std::time::Duration;

use crossterm::event::{self, Event};
use my_hdlc::pc_command::ManualInput;

pub fn keyboard_trimming(keyboard_trim: &mut ManualInput) {
    while event::poll(Duration::from_millis(0)).unwrap() {
        if let Event::Key(key) = event::read().unwrap() {
            match key.code {
                // Lift trim
                // KeyCode::Char('a') => keyboard_trim.get_lift() += 0.01, //throttle up
                // KeyCode::Char('z') => keyboard_trim.get_lift() -= 0.01, //throttle down
                //
                // // Roll trim
                // KeyCode::Right => keyboard_trim.get_roll() -= 0.02, //roll down  right arrow key
                // KeyCode::Left => keyboard_trim.get_roll() += 0.02,  //roll up     left arrow key
                //
                // // Pitch trim
                // KeyCode::Char('i') => keyboard_trim.get_pitch() += 0.02, // pitch up  down arrow key
                // KeyCode::Char('k') => keyboard_trim.get_pitch() -= 0.02, // pitch down up arrow key
                //
                // // Yaw trim
                // KeyCode::Char('q') => keyboard_trim.get_yaw() -= 0.02, //yaw down
                // KeyCode::Char('w') => keyboard_trim.get_yaw() += 0.02, //yaw up
                _ => {}
            }
        }
    }
}
