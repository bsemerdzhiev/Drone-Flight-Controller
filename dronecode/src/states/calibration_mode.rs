use crate::filters::dmp_readings::DmpReadings;
use crate::filters::pressure_filter::PressureSensor;
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
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::block;
use tudelft_quadrupel::mpu::{
    read_dmp_bytes, read_raw,
    structs::{Accel, Gyro},
};
use tudelft_quadrupel::uart::send_bytes;

pub struct FSMCalibration {}

impl FSMControl for FSMCalibration {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read new sample
        ctx.calibration_state.read_new_sample();

        if ctx.calibration_state.should_finish() {
            ctx.calibration_state.finalize_calibration();

            ctx.kalman_position.calibration_offset = (
                ctx.calibration_state.accelerometer_offset,
                ctx.calibration_state.gyro_offset,
            );

            ctx.dmp_filter.calibration_offset_raw_read = ctx.calibration_state.gyro_offset;
            ctx.dmp_filter.calibration_offset = ctx.calibration_state.ypr_offset;

            ctx.pressure_sensor_filter.reset();

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
