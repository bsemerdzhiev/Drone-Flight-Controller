use micromath::F32Ext;

pub const KP_ROLL: f32 = 1.0;
pub const KP_PITCH: f32 = 1.0;

/// Convert quaternion (as f32 components) to roll & pitch
pub fn quaternion_to_roll_pitch(w: f32, x: f32, y: f32, z: f32) -> (f32, f32) {
    // Euler angle equations
    let roll = f32::atan2(2.0 * (w * x + y * z),1.0 - 2.0 * (x * x + y * y));

    let sinp = 2.0 * (w * y - z * x);
    let sinp = sinp.clamp(-1.0, 1.0);  // Clamp necessary since rounding floating point can produce NaN on hardware
    let pitch = sinp.asin();

    (roll, pitch)
}

pub fn roll_controller(desired: f32, measured: f32) -> f32 {
    KP_ROLL * (desired - measured)
}

pub fn pitch_controller(desired: f32, measured: f32) -> f32 {
    KP_PITCH * (desired - measured)
}


pub fn compute_motor_speeds(z_lift: f32,n_yaw: f32,m_pitch: f32,l_roll: f32) -> [u16; 4] {

    /// Motor numbering:
    /// 0 = Front
    /// 1 = Right
    /// 2 = Back
    /// 3 = Left

    let m0 = z_lift - m_pitch + n_yaw; // Front motor
    let m2 = z_lift + m_pitch + n_yaw; // Back motor

    let m1 = z_lift - l_roll - n_yaw;  // Right motor
    let m3 = z_lift + l_roll - n_yaw;  // Left motor

    // Clamp motor speeds just in case, 800 as reasonable max cap defined in quadrupel library
    [
        m0.clamp(0.0, 800.0) as u16,
        m1.clamp(0.0, 800.0) as u16,
        m2.clamp(0.0, 800.0) as u16,
        m3.clamp(0.0, 800.0) as u16,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    // Roll controller tests
    // -------------------------

    #[test]
    fn roll_zero_error_gives_zero_torque() {
        // Zero error -> zero torque
        let torque = roll_controller(0.0, 0.0);
        assert_eq!(torque, 0.0);
    }

    #[test]
    fn roll_positive_error_gives_positive_torque() {
        // sign of error = sign of torque
        let torque = roll_controller(1.0, 0.0);
        assert_eq!(torque, KP_ROLL * 1.0);
    }

    #[test]
    fn roll_negative_error_gives_negative_torque() {
        // sign of error = sign of torque
        let torque = roll_controller(0.0, 1.0);
        assert_eq!(torque, -KP_ROLL * 1.0);
    }

    // Pitch controller tests
    // -------------------------

    #[test]
    fn pitch_zero_error_gives_zero_torque() {
        // Zero error -> zero torque
        let torque = pitch_controller(0.0, 0.0);
        assert_eq!(torque, 0.0);
    }

    #[test]
    fn pitch_positive_error_gives_positive_torque() {
        // sign of error = sign of torque
        let torque = pitch_controller(1.0, 0.0);
        assert_eq!(torque, KP_PITCH * 1.0);
    }

    // Motor mixing tests
    // -------------------------

    #[test]
    fn motor_hover_no_torque() {
        // No torque -> drone hovers equally
        let motors = compute_motor_speeds(
            200.0,
            0.0,
            0.0,
            0.0,
        );

        assert_eq!(motors, [200, 200, 200, 200]);
    }

    #[test]
    fn pitch_forward_increases_back_motor() {
        let motors = compute_motor_speeds(
            200.0,
            0.0,
            10.0,
            0.0,
        );

        // Front = Z - M (lift - pitch torque)
        // Back  = Z + M (lift + pitch torque)
        assert_eq!(motors[0], 190); // front decreases
        assert_eq!(motors[2], 210); // back increases
    }

    #[test]
    fn roll_right_increases_left_motor() {
        let motors = compute_motor_speeds(
            200.0,
            0.0,
            0.0,
            10.0,
        );

        // Right = Z - L (lift - roll torque)
        // Left  = Z + L (lift + roll torque)
        assert_eq!(motors[1], 190); // right decreases
        assert_eq!(motors[3], 210); // left increases
    }

    #[test]
    fn motor_clamping_works() {
        // make sure motor speeds don't exceed the limit
        let motors = compute_motor_speeds(
            900.0,
            0.0,
            0.0,
            0.0,
        );

        assert_eq!(motors, [800, 800, 800, 800]);
    }
}