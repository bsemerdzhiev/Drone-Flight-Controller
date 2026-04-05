use core::time::Duration;

use crate::filters::dmp_readings::DmpReadings;
use crate::states::calibration_mode::FSMCalibration;
use crate::states::fsm_base_class::FSMControl;
use crate::states::full_control::FSMFullControl;
use crate::states::manual_mode::FSMManual;
use crate::states::panic_mode::FSMPanic;
use crate::states::raw_sensor_full_control::FSMRawFullControl;
use crate::states::state_structures::state_context::StateContext;
use crate::states::wireless_mode::FSMWireless;
use crate::states::yaw_control::FSMYaw;
use crate::util::pid_controller::PIDController;
use alloc::boxed::Box;
use fixed::types::{I16F16, I26F6, I4F28};
use my_hdlc::command::DeviceCommand;
use my_hdlc::{command::FSMState, HdlcTransceiver};
use my_hdlc::{telemetry_data::*, MESSAGE_SIZE, STUFFED_MESSAGE_SIZE};
use postcard::from_bytes;
use tudelft_quadrupel::flash::{flash_chip_erase, flash_read_bytes};
use tudelft_quadrupel::led::Red;
use tudelft_quadrupel::led::{Green, Yellow};
use tudelft_quadrupel::motor::{self, *};
use tudelft_quadrupel::time::Instant;
use tudelft_quadrupel::uart::send_bytes;

const SEND_LOG_TIMER: Duration = Duration::from_millis(10);

pub struct FSMSafe {}

impl FSMControl for FSMSafe {
    fn run_state_loop(self: Box<Self>, ctx: &mut StateContext) -> Box<dyn FSMControl> {
        set_motors([0, 0, 0, 0]);

        if !ctx.is_wireless && !ctx.wireless_log.forced_message {
            if ctx.flash_head != ctx.flash_tail {
                send_flash_data(ctx);
            } else if ctx.flash_head != 0 {
                Yellow.on();
                _ = flash_chip_erase();
                Yellow.off();

                ctx.flash_head = 0;
                ctx.flash_tail = 0;
            }
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
                    pid_controller: Box::new(PIDController::<I16F16, I16F16>::new()),
                })
            }
            FSMState::FullControlMode => {
                return Box::new(FSMFullControl {
                    pid_controller: Box::new(PIDController::<I16F16, I16F16>::new()),
                })
            }
            FSMState::RawSensorsFullControlMode => {
                return Box::new(FSMRawFullControl {
                    pid_controller: Box::new(PIDController::<I16F16, I16F16>::new()),
                })
            }
            FSMState::WirelessMode => return Box::new(FSMWireless {}),

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

fn send_flash_data(ctx: &mut StateContext) {
    if Instant::now().duration_since(ctx.wireless_log.last_sent_logged_message) < SEND_LOG_TIMER {
        return;
    }
    Green.on();
    ctx.wireless_log.last_sent_logged_message = Instant::now();

    let mut buffer = [0u8; (STUFFED_MESSAGE_SIZE) as usize];
    let mut msg = ([0u8; STUFFED_MESSAGE_SIZE], 0usize);

    _ = flash_read_bytes(ctx.flash_head as u32, &mut buffer).unwrap();
    ctx.flash_head += STUFFED_MESSAGE_SIZE;

    let data: DeviceCommand = from_bytes(&buffer).unwrap();
    let cmd = data;
    msg = ctx.trv.write_structure(&cmd);

    send_bytes(&msg.0[0..msg.1]);

    Green.off();
}
