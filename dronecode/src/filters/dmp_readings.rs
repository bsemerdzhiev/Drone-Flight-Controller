use crate::{filters::sensors_handler::ImuHandler, util::yaw_pitch_roll::YawPitchRoll};
use tudelft_quadrupel::{
    mpu::{
        read_dmp_bytes,
        structs::{Accel, Gyro},
    },
    time::Instant,
};

/*
* Reads IMU data from the DMP.
* Data is read in Quaternions
* and then parsed into YawPitchRoll(euler angles)
*
* Since we are interested in the rate of change,
* store the previous reading along with the time
* it was taken to compute the rate of change per a second.
*/
pub struct DmpReadings {
    last_sampled_time: Option<Instant>,
    last_sample: Option<YawPitchRoll>,
}

impl DmpReadings {
    pub fn new() -> Self {
        DmpReadings {
            last_sampled_time: None,
            last_sample: None,
        }
    }
}

impl ImuHandler for DmpReadings {
    fn get_reading(&mut self) -> Option<YawPitchRoll> {
        let sampled_dmp_res = read_dmp_bytes();

        if sampled_dmp_res.is_err() {
            return self.last_sample;
        }
        let sampled_quaternion = sampled_dmp_res.unwrap();

        let sampled_yaw_pitch_roll = YawPitchRoll::from(sampled_quaternion);

        if self.last_sampled_time.is_none() {
            self.last_sampled_time = Some(Instant::now());
            self.last_sample = Some(sampled_yaw_pitch_roll);

            return self.last_sample;
        }
        let current_time: Instant = Instant::now();

        let passed_time = current_time
            .duration_since(self.last_sampled_time.unwrap())
            .as_secs_f32();

        // derive rate
        let calculated_rate =
            sampled_yaw_pitch_roll.calculate_rate_per_sec(self.last_sample.unwrap(), passed_time);

        self.last_sampled_time = Some(current_time);
        self.last_sample = Some(sampled_yaw_pitch_roll);

        return Some(calculated_rate);
    }
    fn append_new_reading(&mut self, input: (Accel, Gyro)) {}
}
