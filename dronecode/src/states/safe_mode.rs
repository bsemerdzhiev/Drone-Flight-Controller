use crate::filters::dmp_readings::DmpReadings;
use crate::states::calibration_mode::FSMCalibration;
use crate::states::fsm_base_class::FSMControl;
use crate::states::full_control::FSMFullControl;
use crate::states::manual_mode::FSMManual;
use crate::states::panic_mode::FSMPanic;
use crate::states::state_structures::state_context::StateContext;
use crate::states::yaw_control::FSMYaw;
use crate::util::pid_controller::PIDController;
use alloc::boxed::Box;
use my_hdlc::command::DeviceCommand;
use my_hdlc::telemetry_data::*;
use my_hdlc::{command::FSMState, pc_command::ManualInput, HdlcTransceiver};
use postcard::from_bytes;
use tudelft_quadrupel::flash::{flash_chip_erase, flash_read_bytes};
use tudelft_quadrupel::led::Red;
use tudelft_quadrupel::led::{Green, Yellow};
use tudelft_quadrupel::motor::{self, *};
use tudelft_quadrupel::uart::send_bytes;
pub struct FSMSafe {}

impl FSMControl for FSMSafe {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        set_motors([0, 0, 0, 0]);
        if *ctx.flash_head != *ctx.flash_tail {
            send_flash_data(ctx.flash_tail, ctx.trv);
            *ctx.flash_tail += TELEMETERY_DATA_SIZE;
        } else if *ctx.flash_tail != 0 {
            Yellow.on();
            _ = flash_chip_erase();
            Yellow.off();
            *ctx.flash_head = 0;
            *ctx.flash_tail = 0;
        }

        return self;
    }
    fn step(self: Box<Self>, next_state: FSMState, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        if next_state != FSMState::SafeMode {
            Red.off();
        }
        match next_state {
            FSMState::CalibrationMode => {
                if is_throttle_zero() {
                    ctx.calibration_state.reset();

                    return Box::new(FSMCalibration {});
                }
                return self;
            }
            FSMState::ManualMode => return Box::new(FSMManual {}),
            FSMState::YawControl => {
                return Box::new(FSMYaw {
                    imu_sampler: Box::new(DmpReadings::new(ctx.calibration_state.ypr_offset)),
                    pid_controller: Box::new(PIDController::new()),
                })
            }
            FSMState::FullControlMode => {
                return Box::new(FSMFullControl {
                    imu_sampler: Box::new(DmpReadings::new(ctx.calibration_state.ypr_offset)),
                    pid_controller: Box::new(PIDController::new()),
                })
            }
            FSMState::PanicMode => return Box::new(FSMPanic {}),
            _ => self,
        }
    }
    fn get_state(&self) -> FSMState {
        return FSMState::SafeMode;
    }
}

fn is_throttle_zero() -> bool {
    let speed = get_motors();
    return speed[0] == 0 && speed[1] == 0 && speed[2] == 0 && speed[3] == 0;
}
fn send_flash_data(flash_tail: &mut u32, my_hdlc: &mut HdlcTransceiver) {
    let mut buffer = [0u8; (TELEMETERY_DATA_SIZE + 50) as usize];
    Yellow.on();
    _ = flash_read_bytes(*flash_tail, &mut buffer);
    Yellow.off();
    Green.on();
    let data: TelemetryData = from_bytes(&buffer).unwrap();
    let cmd = DeviceCommand::Telemetry(data);
    // send_bytes(&buffer);/
    let msg = my_hdlc.write_structure(&cmd);
    send_bytes(&msg.0[0..msg.1]);
    Green.off();
}
