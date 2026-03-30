use crate::filters::dmp_readings::DmpReadings;
use crate::states::full_control::FSMFullControl;
use crate::states::manual_mode::FSMManual;
use crate::states::panic_mode::FSMPanic;
use crate::states::state_structures::state_context::StateContext;
use crate::states::yaw_control::FSMYaw;
use crate::states::{fsm_base_class::FSMControl, safe_mode::FSMSafe};
use crate::util::yaw_pitch_roll::YawPitchRoll;

use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::command::{DebugCalibration, DeviceCommand};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::block;
use tudelft_quadrupel::mpu::{
    read_dmp_bytes, read_raw,
    structs::{Accel, Gyro},
};
use tudelft_quadrupel::uart::send_bytes;

pub struct FSMCalibration {}

impl FSMControl for FSMCalibration {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        let (accel, gyro) = read_raw().unwrap();
        let dmp_data = block!(read_dmp_bytes()); //

        let ypr = if (dmp_data.is_ok()) {
            YawPitchRoll::from(dmp_data.unwrap())
        } else {
            YawPitchRoll::new()
        }; //
           // let ypr = match read_dmp_bytes() {
           //     Ok(quaternion) => YawPitchRoll::from(quaternion),
           //     Err(_) => YawPitchRoll::new(),
           // };

        // read new sample
        ctx.calibration_state.read_new_sample(accel, gyro, ypr);

        if ctx.calibration_state.should_finish() {
            ctx.calibration_state.finalize_calibration();
            let msg = ctx
                .trv
                .write_structure(&DeviceCommand::DebugCalibration(DebugCalibration {
                    ypr_offset: [
                        ctx.calibration_state.ypr_offset.yaw,
                        ctx.calibration_state.ypr_offset.pitch,
                        ctx.calibration_state.ypr_offset.roll,
                    ],
                }));
            send_bytes(&msg.0[0..msg.1]);

            return Box::new(FSMSafe {});
        }
        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            _ => return self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::CalibrationMode;
    }
}
