use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::state_context::StateContext;
use crate::states::{fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe};
use crate::util::pid_controller::{add_trims, ControllerFlags, PIDController, K_D, K_I, K_P};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;

use alloc::boxed::Box;
use fixed::types::{I16F16, I26F6, I4F28};
use my_hdlc::command::{DebugRpms, DebugYawPitchRoll, DeviceCommand, DroneInfo, FSMState};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::mpu::is_dmp_enabled;
use tudelft_quadrupel::uart::send_bytes;

pub struct FSMYaw {
    pub pid_controller: Box<PIDController<I16F16, I16F16>>,
}

impl FSMControl for FSMYaw {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data

        let input: YawPitchRoll<I16F16, I16F16> = ctx.dmp_filter.get_reading::<I16F16, I16F16>();

        // let mut target: YawPitchRoll =
        // YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());

        let mut target: YawPitchRoll<I16F16, I16F16> = *ctx.input_as_ypr;

        let (k_p, k_i, k_d) = add_trims(&ctx.input_from_controller);

        // calculate the error correction
        let correction = self.pid_controller.compute_pid_correction(
            input,
            target,
            k_p,
            k_i,
            k_d,
            ControllerFlags::AddP as u8,
        );

        target.yaw -= correction.yaw;
        actuate_motors_with_rates(&target, ctx.input_as_ypr.lift);

        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::YawControl;
    }
}
