use crate::calibration_state::CalibrationState;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_context::StateContext;
use crate::states::{fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe};
use crate::util::pid_controller::{self, ControllerFlags, PIDController};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;

use alloc::boxed::Box;
use my_hdlc::command::FSMState;
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;

pub struct FSMYaw {
    pub imu_sampler: Box<dyn ImuHandler>,
    pub pid_controller: Box<PIDController>,
}

impl FSMControl for FSMYaw {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data
        let input_opt: Option<YawPitchRoll> = self.imu_sampler.get_reading();

        if (input.is_none() || ctx.input_from_controller.is_none()) {
            return self;
        }

        let input = input_opt.unwrap();
        // reading from joystick
        let target: YawPitchRoll =
            YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());

        // calculate the error correction
        let correction = self.pid_controller.compute_pid_correction(
            input,
            target,
            k_p,
            k_i,
            k_d,
            ControllerFlags::AddP as u8,
        );

        // add to current input
        ctx.input_from_controller
            .unwrap()
            .increment_yaw(correction.yaw as i32);
        ctx.input_from_controller
            .unwrap()
            .increment_pitch(correction.pitch as i32);
        ctx.input_from_controller
            .unwrap()
            .increment_roll(correction.roll as i32);

        // output to motors
        actuate_motors_with_rates(&ctx.input_from_controller.unwrap(), ctx.trv);

        *ctx.input_from_controller = None;

        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::SafeMode => Box::new(FSMSafe {}),
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::YawControl;
    }
}
