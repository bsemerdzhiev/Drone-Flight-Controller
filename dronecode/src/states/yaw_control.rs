use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::state_context::StateContext;
use crate::states::{fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe};
use crate::util::pid_controller::{ControllerFlags, PIDController, K_D, K_I, K_P};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;

use alloc::boxed::Box;
use my_hdlc::command::{DebugRpms, DebugYawPitchRoll, DeviceCommand, DroneInfo, FSMState};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::mpu::is_dmp_enabled;
use tudelft_quadrupel::uart::send_bytes;

pub struct FSMYaw {
    pub pid_controller: Box<PIDController>,
}

impl FSMControl for FSMYaw {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data

        let input_opt: Option<YawPitchRoll> = ctx.dmp_filter.get_reading();

        if (input_opt.is_none() || ctx.input_from_controller.is_none()) {
            return self;
        }

        let target: YawPitchRoll =
            YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());

        let input = input_opt.unwrap();

        let mut k_p: [f32; 4] = K_P;
        let mut k_i: [f32; 4] = K_I;
        let mut k_d: [f32; 4] = K_D;

        k_p[0] += ctx.input_from_controller.as_ref().unwrap().yaw_p_trim;

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
            .as_mut()
            .unwrap()
            .increment_yaw(correction.yaw as i32);
        // output to motors
        actuate_motors_with_rates(&ctx.input_from_controller.as_ref().unwrap(), ctx.trv);

        // *ctx.input_from_controller = None;

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
