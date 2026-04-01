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
        pid_controller::{add_trims, ControllerFlags, PIDController, K_D, K_I, K_P},
        rpm_calculator::{actuate_motors_with_rates, THRESHOLD_LIFT},
        yaw_pitch_roll::YawPitchRoll,
    },
};

pub struct FSMHeightControl {
    pub pid_controller: Box<PIDController>,
    pub prev_state: Box<dyn FSMControl>,

    pub initial_lift: f32,
    pub initial_pressure: f32,
}

impl FSMControl for FSMHeightControl {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // send chosen height
        ctx.pid_info.selected_height = self.initial_pressure;

        // read sensor data
        // let mut input: YawPitchRoll = ctx.kalman_position.get_reading();
        let mut input: YawPitchRoll = ctx.dmp_filter.get_reading();

        if ctx.input_from_controller.is_none() {
            return self;
        }

        // if lift is changed, return to previous state
        if (ctx.input_from_controller.as_ref().unwrap().get_lift() - self.initial_lift).abs()
            > f32::EPSILON
        {
            return self.prev_state;
        }

        input.pressure = ctx.pressure_sensor_filter.get_reading();

        let (k_p, k_i, k_d) = add_trims(&ctx.input_from_controller.as_ref().unwrap());

        // the target
        let mut target: YawPitchRoll =
            YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());

        target.pressure = self.initial_pressure;
        target.lift = 0f32;

        // calculate the error correction
        let correction = self.pid_controller.compute_pid_correction(
            input,
            target,
            k_p,
            k_i,
            k_d,
            ControllerFlags::AddP as u8 | ControllerFlags::AddD as u8 | ControllerFlags::AddI as u8,
        );

        target.lift += correction.lift;
        target.yaw -= correction.yaw;
        target.roll += correction.roll;
        target.pitch += correction.pitch;

        // output to motors
        // raw_lift is set to threshold lift, as we want to hover at the same position
        actuate_motors_with_rates(&target, THRESHOLD_LIFT);

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
