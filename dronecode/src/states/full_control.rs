use my_hdlc::command::FSMState;
use tudelft_quadrupel::barometer::read_pressure;
use crate::states::fsm_base_class::FSMControl;
use crate::states::height_control::FSMHeightControl;
use crate::states::panic_mode::FSMPanic;
use crate::states::safe_mode::FSMSafe;
use crate::states::state_structures::state_context::StateContext;
use crate::filters::dmp_readings::DmpReadings;
use crate::filters::sensors_handler::ImuHandler;
use crate::util::pid_controller::{ControllerFlags, PIDController};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use alloc::boxed::Box;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu;

// TODO: Tune the parameters
// Order of parameters: Yaw - Pitch - Roll

const K_P: [f32; 4] = [20f32, 1000f32, 1000f32, 0f32];
const K_I: [f32; 4] = [0f32, 0f32, 0f32, 0f32];
const K_D: [f32; 4] = [0f32, 0f32, 0f32, 0f32];

pub struct FSMFullControl {
    pub imu_sampler: Box<dyn ImuHandler>,
    pub pid_controller: Box<PIDController>,
}

impl FSMControl for FSMFullControl {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data
        let input_opt: Option<YawPitchRoll> = self.imu_sampler.get_reading();

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
            .roll_pitch_p_trim;
        k_d[2] += ctx
            .input_from_controller
            .as_ref()
            .unwrap()
            .roll_pitch_p_trim;
        ctx.live_controller_values.p_yaw = k_p[0];
        ctx.live_controller_values.p_pitch = k_p[1];
        ctx.live_controller_values.p_roll = k_p[2];

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
        ctx.input_from_controller
            .as_mut()
            .unwrap()
            .increment_pitch(correction.pitch as i32);
        ctx.input_from_controller
            .as_mut()
            .unwrap()
            .increment_roll(correction.roll as i32);

        // output to motors
        actuate_motors_with_rates(&ctx.input_from_controller.as_ref().unwrap(), ctx.trv);

        *ctx.input_from_controller = None;

        return self;
    }

    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        match next_state {
            FSMState::PanicMode => Box::new(FSMPanic {}),
            FSMState::SafeMode => Box::new(FSMSafe {}),
            FSMState::HeightControlMode => Box::new(FSMHeightControl {
                imu_sampler: Box::new(DmpReadings::new(ctx.calibration_state.ypr_offset)),
                pid_controller: Box::new(PIDController::new()),

                prev_state: self,
                initial_pressure: read_pressure() as f32,
            }),

            _ => self,
        }
    }

    fn get_state(&self) -> FSMState {
        return FSMState::FullControlMode;
    }
}
