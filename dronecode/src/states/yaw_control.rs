use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::state_context::StateContext;
use crate::states::{fsm_base_class::FSMControl, panic_mode::FSMPanic, safe_mode::FSMSafe};
use crate::util::pid_controller::{self, ControllerFlags, PIDController};
use crate::util::rpm_calculator::actuate_motors_with_rates;
use crate::util::yaw_pitch_roll::YawPitchRoll;

use alloc::boxed::Box;
use my_hdlc::command::{DebugRpms, DeviceCommand, DroneInfo, FSMState};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::mpu::is_dmp_enabled;
use tudelft_quadrupel::uart::send_bytes;

// TODO: Tune the parameters
const K_P: [i32; 3] = [0, 0, 0];
const K_I: [i32; 3] = [0, 0, 0];
const K_D: [i32; 3] = [0, 0, 0];

pub struct FSMYaw {
    pub imu_sampler: Box<dyn ImuHandler>,
    pub pid_controller: Box<PIDController>,
}

impl FSMControl for FSMYaw {
    fn run_state_loop(mut self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        // read sensor data
        let input_opt: Option<YawPitchRoll> = self.imu_sampler.get_reading();
        let to_write = ctx
            .trv
            .write_structure(&DeviceCommand::DebugRpms(DebugRpms::new(&[
                input_opt.is_none() as u16,
                ctx.input_from_controller.is_none() as u16,
                is_dmp_enabled() as u16,
                0,
            ])));

        send_bytes(&to_write.0[0..to_write.1]);

        // reading from joystick
        if (input_opt.is_none() || ctx.input_from_controller.is_none()) {
            return self;
        }
        let target: YawPitchRoll =
            YawPitchRoll::from_manual_input(ctx.input_from_controller.as_ref().unwrap());

        let input = input_opt.unwrap();

        // calculate the error correction
        let correction = self.pid_controller.compute_pid_correction(
            input,
            target,
            K_P,
            K_I,
            K_D,
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
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::YawControl;
    }
}
