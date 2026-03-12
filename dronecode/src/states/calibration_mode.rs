use crate::calibration_state::Axis;
use crate::calibration_state::CalibrationState;
use crate::filters::dmp_readings::DmpReadings;
use crate::states::full_control::FSMFullControl;
use crate::states::manual_mode::FSMManual;
use crate::states::panic_mode::FSMPanic;
use crate::states::state_context::StateContext;
use crate::states::yaw_control::FSMYaw;
use crate::states::{fsm_control_trait::FSMControl, safe_mode::FSMSafe};

use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::mpu::{
    read_raw,
    structs::{Accel, Gyro},
};

pub struct FSMCalibration {}

impl From<Accel> for Axis {
    fn from(a: Accel) -> Self {
        Axis {
            x: a.x,
            y: a.y,
            z: a.z,
        }
    }
}

impl From<Gyro> for Axis {
    fn from(g: Gyro) -> Self {
        Axis {
            x: g.x,
            y: g.y,
            z: g.z,
        }
    }
}

impl FSMControl for FSMCalibration {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        let (accel, gyro) = read_raw().unwrap();
        ctx.calibration_state
            .accumulate_calibration(Axis::from(accel), Axis::from(gyro));
        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::SafeMode => Box::new(FSMSafe {}),
            _ => return self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::CalibrationMode;
    }
}
