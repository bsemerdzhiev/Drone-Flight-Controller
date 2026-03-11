use core::time::Duration;

use crate::calibration_state::CalibrationState;
use crate::states::manual_mode::FSMManual;
use crate::states::safe_mode::FSMSafe;
use crate::states::FSM_control_trait::FSMControl;
use crate::telemetry_read::TelemetryRead;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;

use my_hdlc::command::{self, DeviceCommand, DroneInfo};

use my_hdlc::pc_command::ManualInput;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::{Blue, Green};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use my_hdlc::{HdlcTransceiver, STUFFED_MESSAGE_SIZE};

const UART_BUF_SIZE: usize = my_hdlc::BUFFER_SIZE;

const SHOULD_CHECK_BATTERY_LEVEL: bool = false;
const MIN_BAT_LEVEL: u16 = 1050;

pub fn main_loop() -> ! {
    set_tick_frequency(100);

    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();

    let mut last_instant = Instant::now();

    let mut current_state: &dyn FSMControl = &FSMSafe;

    let mut iterations_without_message = 0u32;

    let mut battery_panic = false;

    let mut uart_buf = [0u8; UART_BUF_SIZE];

    let mut calibration_state: CalibrationState = CalibrationState::new();
    let mut received_manual_input: ManualInput = ManualInput::default();
    let mut has_received_input: bool;

    for i in 0.. {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last_instant);
        last_instant = now;
        has_received_input = false;

        // Check battery level and switch to panic
        let bat_level = read_battery();
        if SHOULD_CHECK_BATTERY_LEVEL && bat_level < MIN_BAT_LEVEL {
            current_state =
                current_state.step(command::FSMState::PanicMode, &mut calibration_state);
            battery_panic = true;
        } else if battery_panic && bat_level >= MIN_BAT_LEVEL {
            current_state = current_state.step(command::FSMState::SafeMode, &mut calibration_state);
            battery_panic = false;
        }

        // Read Uart Buff
        let num_received = receive_bytes(&mut uart_buf[0..transceiver.remaining_bytes]);

        if num_received != 0usize && !battery_panic {
            transceiver.add_bytes(&uart_buf[..num_received]);
            let deserialized_command = transceiver.read_structure::<DeviceCommand>();
            if let Some(command) = deserialized_command {
                match command {
                    DeviceCommand::ChangeMode(new_mode) => {
                        current_state = current_state.step(new_mode, &mut calibration_state);
                        send_ack(&mut transceiver);
                    }
                    DeviceCommand::ManualInput(manual_input) => {
                        received_manual_input = manual_input;
                        has_received_input = true;
                    }
                    _ => continue,
                }
                iterations_without_message = 0;
            }
        } else if !battery_panic {
            iterations_without_message += 1;
            if iterations_without_message == 50 {
                current_state =
                    current_state.step(command::FSMState::PanicMode, &mut calibration_state);
            }
        }

        current_state = current_state.run_control_loop(
            &mut calibration_state,
            &received_manual_input,
            &mut has_received_input,
            &mut transceiver,
        );
        if i % 100 == 0 {
            send_drone_data(&mut transceiver, dt);
            Green.off();
        }

        let to_write = transceiver.write_structure(&DeviceCommand::DroneInfo(DroneInfo::new(
            current_state.get_state(),
            read_battery(),
        )));

        send_bytes(&to_write.0[0..to_write.1]);

        wait_for_next_tick();
    }
    unreachable!();
}

fn send_drone_data(transceiver: &mut HdlcTransceiver, dt: Duration) {
    let data = TelemetryRead::read_telemetry(dt);
    let cmd: DeviceCommand = DeviceCommand::Telemetry(data);
    Green.on();

    let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = transceiver.write_structure(&cmd);
    send_bytes(&msg.0[0..msg.1]);
}

fn send_ack(transceiver: &mut HdlcTransceiver) {
    let ack_cmd = DeviceCommand::Ack;
    let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = transceiver.write_structure(&ack_cmd);
    send_bytes(&msg.0[0..msg.1]);
}
