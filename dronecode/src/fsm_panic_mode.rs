use crate::control_trait::FSMControl;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::*;
use tudelft_quadrupel::time::*;
use tudelft_quadrupel::led::Led::Green;

pub struct FSMPanic {
    time: Instant,
}

impl FSMPanic {
    pub fn new() -> Self {
        Green.toggle();
        Self {
            time: Instant::now(),
        }
        
    }
}

impl FSMControl for FSMPanic {
    fn run_control_loop(&self) {
        let now = Instant::now();
        let ns = now.ns_since_start() - self.time.ns_since_start();
        let ms = ns / 1000000;

        let total_duration = 1000; // in ms
        let panic_motor_max = 100; //should be adjusted as needed

        let factor = if ms >= total_duration {
            0.0
        } else {
            1.0 - (ms as f32 / total_duration as f32)
        };

        let motor_max = (panic_motor_max as f32 * factor) as u16;
        set_motor_max(motor_max);

        if ms >= total_duration {
            Green.toggle();
        }
    }
    fn step(&self, next_state: FSMState) -> &dyn FSMControl {
        match next_state {
            FSMState::SafeMode => return &FSMSafe,
            FSMState::PanicMode => return &FSMPanic,
            _ => {} // can only stay in panic or go to safe
        }
    }
}


