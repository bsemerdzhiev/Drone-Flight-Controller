use alloc::boxed::Box;
use fixed::types::{I16F16, I26F6, I4F28};
use my_hdlc::command::{DebugYawPitchRoll, DeviceCommand, FSMState};
use tudelft_quadrupel::{
    barometer::read_pressure, mpu::read_raw, once_cell, time::Instant, uart::send_bytes,
};

use crate::{
    filters::sensors_handler::ImuHandler,
    states::{
        fsm_base_class::FSMControl, full_control::FSMFullControl, panic_mode::FSMPanic,
        raw_sensor_full_control::FSMRawFullControl, safe_mode::FSMSafe,
        state_structures::state_context::StateContext,
    },
    util::{
        pid_controller::{add_trims, ControllerFlags, PIDController, K_D, K_I, K_P},
        rpm_calculator::{actuate_motors_with_rates, ThresholdLift},
        yaw_pitch_roll::YawPitchRoll,
    },
};

pub struct FSMHeightControl {
    pub pid_controller: Box<PIDController<I16F16, I16F16>>,
    pub prev_state: Box<dyn FSMControl>,

    pub initial_lift: I16F16,
    pub initial_pressure: I16F16,
}

impl FSMControl for FSMHeightControl {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data
        let mut input: YawPitchRoll<I16F16, I16F16> = match self.prev_state.get_state() {
            FSMState::FullControlMode => ctx.dmp_filter.get_reading::<I16F16, I16F16>(),
            _ => ctx.kalman_position.get_reading::<I16F16, I16F16>(),
        };

        // if lift is changed, return to previous state
        if (ctx.input_as_ypr.lift != self.initial_lift) {
            return self.prev_state;
        }

        input.pressure = ctx.pressure_sensor_filter.get_reading();

        let (k_p, k_i, k_d) = add_trims(&ctx.trim_input);

        let mut target: YawPitchRoll<I16F16, I16F16> = *ctx.input_as_ypr;
        target.pressure = self.initial_pressure;

        target.lift = I16F16::from_num(0);

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
        actuate_motors_with_rates(&target, ThresholdLift);

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
