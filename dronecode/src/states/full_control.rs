use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

use crate::calibration_state::CalibrationState;
use crate::calibration_state::CalibrationState;
use crate::full_control_logic as logic;
use crate::full_control_logic as logic;
use crate::states::FSM_control_trait::FSMControl;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu;

pub struct FSMFullControl;

impl FSMFullControl {
    pub const fn new() -> Self {
        Self
    }
}

impl FSMControl for FSMFullControl {
    fn run_control_loop(
        &self,
        calibration_state: &mut crate::calibration_state::CalibrationState,
        command: &ManualInput,
        has_received_input: &mut bool,
        my_hdlc: &mut HdlcTransceiver,
    ) -> &dyn FSMControl {
        // Get quaternion from DMP
        // -------------------------------------------------------------
        if let Ok(q) = mpu::read_dmp_bytes() {
            //Control loops only updates when DMP data arrives
            //let count = self.print_counter.fetch_add(1, Ordering::Relaxed);

            // Convert fixed-point -> f32
            let w = q.w.to_num::<f32>();
            let x = q.x.to_num::<f32>();
            let y = q.y.to_num::<f32>();
            let z = q.z.to_num::<f32>();

            // Convert to roll and pitch
            // -------------------------------------------------------------
            let (roll, pitch) = logic::quaternion_to_roll_pitch(w, x, y, z);

            // Desired angles (from RC / keyboard)
            // -------------------------------------------------------------
            // Neutral stick -> 0 rad
            let desired_roll: f32 = 0.0;
            let desired_pitch: f32 = 0.0;

            // Controllers -> torques
            // torque = Kp (proportional gain) x error
            // torque later converted to motor speed differences
            // -------------------------------------------------------------
            let l_roll = logic::roll_controller(desired_roll, roll);
            let m_pitch = logic::pitch_controller(desired_pitch, pitch);

            // Send torques to motor mixer
            // -------------------------------------------------------------
            let z_lift: f32 = 200.0; // no lift value yet so use predefined for now
            let n_yaw: f32 = 0.0; // no yaw control yet
            let motors = logic::compute_motor_speeds(z_lift, n_yaw, m_pitch, l_roll);
            set_motors(motors);

            // // For DEBUG printing
            // if count % 2 == 0 { // After 2 DMP sensor updates do printing

            //     let roll_deg = roll * 57.2958; // Multiplied by this to convert from radians to degrees
            //     let pitch_deg = pitch * 57.2958;

            //     rprintln!("Roll: {:.2}°, Pitch: {:.2}°",roll_deg,pitch_deg);
            //     rprintln!("Motors: {:?}", motors);
            // }
        }
        self
    }

    fn step(
        &self,
        next_state: FSMState,
        _calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {
        match next_state {
            FSMState::FullControlMode => self,
            _ => self, // transition to a different state
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::CalibrationMode;
    }
}
