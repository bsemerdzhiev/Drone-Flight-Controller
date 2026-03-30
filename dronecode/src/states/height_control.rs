use alloc::boxed::Box;
use my_hdlc::command::{DebugYawPitchRoll, DeviceCommand, FSMState};
use tudelft_quadrupel::{
    barometer::read_pressure, mpu::read_raw, once_cell, time::Instant, uart::send_bytes,
};

use crate::{
    filters::sensors_handler::ImuHandler,
    states::{
        fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe,
        state_structures::state_context::StateContext,
    },
    util::{
        pid_controller::{ControllerFlags, PIDController, K_D, K_I, K_P},
        rpm_calculator::actuate_motors_with_rates,
        yaw_pitch_roll::YawPitchRoll,
    },
};

pub struct FSMHeightControl {
    pub pid_controller: Box<PIDController>,
    pub prev_state: Box<dyn FSMControl>,

    pub initial_pressure: f32,
}

impl FSMControl for FSMHeightControl {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        //TODO: Implement going back to the state from which we came

        // read sensor data
        let input_opt: Option<YawPitchRoll> = ctx.kalman_position.get_reading();

        if (input_opt.is_none() || ctx.input_from_controller.is_none()) {
            return self;
        }

        let input = input_opt.unwrap();

        let mut k_p: [f32; 4] = K_P;
        let mut k_i: [f32; 4] = K_I;
        let mut k_d: [f32; 4] = K_D;

        k_p[0] += ctx.input_from_controller.as_ref().unwrap().yaw_p_trim;

        k_p[1] += ctx
            .input_from_controller
            .as_ref()
            .unwrap()
            .roll_pitch_p_trim;
        k_p[2] += ctx
            .input_from_controller
            .as_ref()
            .unwrap()
            .roll_pitch_p_trim;

        k_d[1] += ctx
            .input_from_controller
            .as_ref()
            .unwrap()
            .roll_pitch_d_trim;
        k_d[2] += ctx
            .input_from_controller
            .as_ref()
            .unwrap()
            .roll_pitch_d_trim;

        // the target
        let mut target: YawPitchRoll =
            YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());
        target.pressure = self.initial_pressure;

        // calculate the error correction
        let correction = self.pid_controller.compute_pid_correction(
            input,
            target,
            k_p,
            k_i,
            k_d,
            ControllerFlags::AddP as u8,
        );

        // let to_write =
        //     ctx.trv
        //         .write_structure(&DeviceCommand::DebugYawPitchRoll(DebugYawPitchRoll {
        //             info: [
        //                 correction.lift,
        //                 correction.yaw,
        //                 correction.pitch,
        //                 correction.roll,
        //                 correction.pressure,
        //             ],
        //         }));
        //
        // send_bytes(&to_write.0[0..to_write.1]);
        //
        // add to current input

        target.lift += correction.lift;
        target.yaw += correction.yaw;
        target.roll += correction.yaw;
        target.pitch -= correction.yaw;

        // output to motors
        actuate_motors_with_rates(
            &target,
            ctx.input_from_controller.as_ref().unwrap().get_lift(),
        );

        //*ctx.input_from_controller = None;

        return self;
    }

    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            _ => self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::HeightControlMode;
    }
}
