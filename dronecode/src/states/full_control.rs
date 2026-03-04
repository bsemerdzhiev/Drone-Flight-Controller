use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};

use crate::states::FSM_control_trait::FSMControl;
use crate::calibration_state::{CalibrationState, Axis};
use tudelft_quadrupel::mpu;
use tudelft_quadrupel::mpu::structs::Quaternion;
use my_hdlc::command::FSMState;
use tudelft_quadrupel::motor::set_motors;

/// Proportional gains for pitch and roll
const KP_ROLL: f32 = 1.0;
const KP_PITCH: f32 = 1.0;

/// Full-control FSM
pub struct FSMFullControl;

impl FSMFullControl {
    /// Convert DMP quaternion -> (roll, pitch) in radians
    fn quaternion_to_roll_pitch(q: Quaternion) -> (f32, f32) {
        // fixed-point -> f32
        let w = q.w.to_num::<f32>();
        let x = q.x.to_num::<f32>();
        let y = q.y.to_num::<f32>();
        let z = q.z.to_num::<f32>();

        // Euler angle equations
        let roll = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));

        let pitch = (2.0 * (w * y - z * x)).asin();

        (roll, pitch)
    }

    /// Roll controller (P-controller)
    fn roll_controller(desired: f32, measured: f32) -> f32 {
        KP_ROLL * (desired - measured)
    }

    /// Pitch controller (P-controller)
    fn pitch_controller(desired: f32, measured: f32) -> f32 {
        KP_PITCH * (desired - measured)
    }

    fn motor_mixing(z_lift: f32,n_yaw: f32,m_pitch: f32,l_roll: f32){
        
        let m0 = z_lift - m_pitch + n_yaw; // Front motor
        let m2 = z_lift + m_pitch + n_yaw; // Back motor

        let m1 = z_lift - l_roll - n_yaw;  // Right motor
        let m3 = z_lift + l_roll - n_yaw;  // Left motor

        // Clamp motor speeds just in case, 800 as reasonable max cap defined in quadrupel library
        let m0 = m0.clamp(0.0, 800.0) as u16;
        let m1 = m1.clamp(0.0, 800.0) as u16;
        let m2 = m2.clamp(0.0, 800.0) as u16;
        let m3 = m3.clamp(0.0, 800.0) as u16;

        set_motors([m0,m1,m2,m3]); //Apply correction motor speeds
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
        todo!();
    }
    fn step(
        &self,
        next_state: FSMState,
        _calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {
        match next_state {
            FSMState::FullControl => self,
            _ => self, // transition to a different state
        }
    }

    fn run_control_loop(
        &self,
        _calibration_state: &mut CalibrationState,
    ) -> &dyn FSMControl {

         // Get quaternion from DMP
        // -------------------------------------------------------------
        if let Ok(q) = mpu::read_dmp_bytes(){ //Control loops only updates when DMP data arrives
            // Convert to roll and pitch
            // -------------------------------------------------------------
            let (roll, pitch) = Self::quaternion_to_roll_pitch(q);


            // Desired angles (from RC / keyboard)
            // -------------------------------------------------------------
            // Neutral stick -> 0 rad
            let desired_roll: f32 = 0.0;
            let desired_pitch: f32 = 0.0;

            // Controllers -> torques
            // torque = Kp (proportional gain) x error
            // torque later converted to motor speed differences
            // -------------------------------------------------------------
            let l_roll = Self::roll_controller(desired_roll, roll);
            let m_pitch = Self::pitch_controller(desired_pitch, pitch);


            // Send torques to motor mixer
            // -------------------------------------------------------------
            let z_lift: f32 = 200.0; // no lift value yet so use predefined for now
            let n_yaw: f32 = 0.0;    // no yaw control yet
            Self::motor_mixing(z_lift,n_yaw,m_pitch,l_roll);

            // Stay in full control
        }
        
        self
    }
    fn get_state(&self) -> FSMState {
        return FSMState::CalibrationMode;
    }
}
