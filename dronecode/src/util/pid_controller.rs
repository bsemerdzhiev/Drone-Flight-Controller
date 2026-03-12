use tudelft_quadrupel::time::Instant;

use crate::util::yaw_pitch_roll::YawPitchRoll;

/*
* Selects the type of error correction
* we want to perform
*/
#[repr(u8)]
pub enum ControllerFlags {
    AddP = (1 << 0),
    AddD = (1 << 1),
    AddI = (1 << 2),
}

pub struct PIDController {
    prev_error: YawPitchRoll,
    integration_build_up: YawPitchRoll,
    last_timestamp: Instant,
}

impl PIDController {
    pub fn new() -> Self {
        PIDController {
            prev_error: YawPitchRoll::new(),
            integration_build_up: YawPitchRoll::new(),
            last_timestamp: Instant::now(),
        }
    }

    pub fn compute_pid_correction(
        &mut self,
        input: YawPitchRoll,
        target: YawPitchRoll,
        k_p: i32,
        k_i: i32,
        k_d: i32,
        controller_flags: u8,
    ) -> YawPitchRoll {
        let result = YawPitchRoll::new();
        let calculated_error = (target - input);
        let current_time = Instant::now();
        let delta_t = current_time.duration_since(self.last_timestamp);

        // compute P part
        if (controller_flags & ControllerFlags::AddP) {
            result += calculated_error * k_p;
        }

        // compute D part
        if (controller_flags & ControllerFlags::AddD) {
            result += k_d * ((calculated_error - self.prev_error) / delta_t);
            self.prev_error = calculated_error;
        }

        // compute I part
        if (controller_flags & ControllerFlags::AddI) {
            self.integration_build_up += (calculated_error - self.prev_error) * delta_t;
            result += k_i * self.integration_build_up;
        }
        // update the timestamp
        self.last_timestamp = current_time;

        return result;
    }
}
