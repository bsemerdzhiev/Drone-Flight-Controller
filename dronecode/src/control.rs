use crate::control_trait::FSMControl;
use crate::fsm_manual_mode::FSMManual;
use crate::fsm_safe_mode::FSMSafe;
use core::time::Duration;

use crate::calibration_state::CalibrationState;
use crate::states::safe_mode::FSMSafe;
use crate::states::FSM_control_trait::FSMControl;
use crate::telemetry_read::TelemetryRead;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use my_hdlc::command::{self, Command};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::{Blue, Green};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::nrf51_hal::uart;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use my_hdlc::HdlcTransceiver;

pub fn main_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    let mut op_mode: &dyn FSMControl = &FSMManual;
    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();

    let mut command: Option<Command> = None;

    let mut receive_buffer = [0u8; my_hdlc::BUFFER_SIZE];

    let mut op_mode: &dyn FSMControl = &FSMSafe;
    let mut uart_buf = [0u8; UART_BUF_SIZE];
    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();
    let mut calibration_state: CalibrationState = CalibrationState::new();
    for i in 0.. {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        // Read Uart Buff
        let num_received = receive_bytes(&mut uart_buf);
        if num_received != 0usize {
            transceiver.add_bytes(&uart_buf[..num_received]);
            let deserialized_command = transceiver.read_structure::<DeviceCommand>();
            if let Some(command) = deserialized_command {
                match command {
                    DeviceCommand::ChangeMode(new_mode) => {
                        op_mode = op_mode.step(new_mode, &mut calibration_state);
                        send_ack(&mut transceiver);
                    }
                    _ => continue,
                }
            }
        }

        // control_loop(op_mode);
        op_mode = op_mode.run_control_loop(&mut calibration_state);
        if i % 100 == 0 {
            send_drone_data(&mut transceiver, dt);
            Green.off();
        }

        // Control Loop:
        // Read DeviceCommand and Execute, (if available).
        // Run the current mode's control loop
        // send data if i%100 == 0
        // wait until the timer interrupt goes off again
        // based on the frequency set above

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
    // todo!("Put the data in a struct and send it!");
}

fn send_ack(transceiver: &mut HdlcTransceiver) {
    let ack_cmd = DeviceCommand::Ack;
    let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = transceiver.write_structure(&ack_cmd);
    uart::send_bytes(&msg.0[0..msg.1]);
}
