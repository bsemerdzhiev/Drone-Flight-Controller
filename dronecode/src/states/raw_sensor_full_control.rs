use my_hdlc::command::{DebugYawPitchRoll, DeviceCommand};
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::uart::send_bytes;

use crate::filters::dmp_readings::DmpReadings;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::fsm_base_class::FSMControl;
use crate::states::height_control::FSMHeightControl;
use crate::states::panic_mode::FSMPanic;
use crate::states::safe_mode::FSMSafe;
use crate::states::state_structures::state_context::StateContext;
use crate::util::pid_controller::{add_trims, ControllerFlags, PIDController, K_D, K_I, K_P};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use alloc::boxed::Box;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::{self, read_raw};

// TODO: Tune the parameters
// Order of parameters: Yaw - Pitch - Roll

pub struct FSMRawFullControl {
    pub pid_controller: Box<PIDController>,
}

impl FSMControl for FSMRawFullControl {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data
        let input_opt: Option<YawPitchRoll> = ctx.kalman_position.get_reading();

        if (input_opt.is_none() || ctx.input_from_controller.is_none()) {
            return self;
        }
        let mut target: YawPitchRoll =
            YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());

        let input = input_opt.unwrap();

        let (k_p, k_i, k_d) = add_trims(&ctx.input_from_controller.as_ref().unwrap());
        // calculate the error correction
        let correction = self.pid_controller.compute_pid_correction(
            input,
            target,
            k_p,
            k_i,
            k_d,
            ControllerFlags::AddP as u8,
        );

        target.yaw += correction.yaw;
        target.roll += correction.roll;
        target.pitch += correction.pitch;

        // output to motors
        actuate_motors_with_rates(
            &target,
            ctx.input_from_controller.as_ref().unwrap().get_lift(),
        );

        return self;
    }

    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::HeightControlMode => Box::new(FSMHeightControl {
                pid_controller: Box::new(PIDController::new()),

                prev_state: self,
                initial_pressure: ctx.pressure_sensor_filter.get_reading(),
            }),

            _ => self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::RawSensorsFullControlMode;
    }
}
