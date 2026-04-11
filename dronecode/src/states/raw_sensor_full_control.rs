use fixed::types::{I16F16, I26F6, I4F28};
use my_hdlc::command::{DebugYawPitchRoll, DeviceCommand};
use my_hdlc::{command::FSMState, HdlcTransceiver};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::uart::send_bytes;

use crate::filters::dmp_readings::DmpReadings;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::fsm_base_class::FSMControl;
use crate::states::height_control::FSMHeightControl;
use crate::states::panic_mode::FSMPanic;
use crate::states::safe_mode::FSMSafe;
use crate::states::state_structures::state_context::StateContext;
use crate::util::pid_controller::{ControllerFlags, PIDController};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use alloc::boxed::Box;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::{self, read_raw};

// TODO: Tune the parameters
// Order of parameters: Yaw - Pitch - Roll

pub struct FSMRawFullControl {}

impl FSMControl for FSMRawFullControl {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data
        let input: YawPitchRoll<I16F16, I16F16> =
            ctx.kalman_position.get_reading::<I16F16, I16F16>();

        let mut target: YawPitchRoll<I16F16, I16F16> = *ctx.input_as_ypr;

        // calculate the error correction
        let correction = ctx.pid_controller.compute_pid_correction(
            input,
            target,
            ControllerFlags::AddP as u8 | ControllerFlags::AddD as u8 | ControllerFlags::AddI as u8,
        );

        target.yaw = -correction.yaw;
        target.roll = correction.roll;
        target.pitch = correction.pitch;

        // output to motors
        actuate_motors_with_rates(&target, ctx.input_as_ypr.lift);

        return self;
    }

    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::HeightControlMode => {
                let z = Box::new(FSMHeightControl {
                    prev_state: self,
                    initial_pressure: ctx.pressure_sensor_filter.get_reading(),
                    initial_lift: I16F16::from_num(ctx.input_as_ypr.lift),
                });
                ctx.pid_info.selected_height = z.initial_pressure.to_num::<f32>();
                return z;
            }

            _ => self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::RawSensorsFullControlMode;
    }
}
