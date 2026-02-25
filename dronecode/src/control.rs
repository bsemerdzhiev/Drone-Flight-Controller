use core::time::Duration;

use crate::sensor_state::SensorState;
use crate::states::safe_mode::FSMSafe;
use crate::states::FSM_control_trait::FSMControl;
use crate::telemetry_read::TelemetryRead;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use my_hdlc::command::{DeviceCommand, FSMState};
use my_hdlc::telemetry_data::TelemetryData;
use my_hdlc::{HdlcTransceiver, STUFFED_MESSAGE_SIZE};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{self, receive_bytes, send_bytes};
const UART_BUF_SIZE: usize = 255usize;

pub fn main_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    let mut op_mode: &dyn FSMControl = &FSMSafe;
    let mut uart_buf = [0u8; UART_BUF_SIZE];
    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();
    let mut zero_states: SensorState = SensorState::new();
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
            if let Some(DeviceCommand) = deserialized_command {
                run_command(DeviceCommand);
            }
        }

        // control_loop(op_mode);
        op_mode = op_mode.run_control_loop(&mut zero_states);
        if i % 100 == 0 {
            send_drone_data(&mut transceiver, dt);
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

fn run_command(command: DeviceCommand) {
    todo!("Execute Commands!");
}

fn send_drone_data(transiver: &mut HdlcTransceiver, dt: Duration) {
    let data = TelemetryRead::read_telemetry(dt);
    let cmd: DeviceCommand = DeviceCommand::Telemetry(data);
    let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = transiver.write_structure(&cmd);
    send_bytes(&msg.0[0..msg.1]);
    // todo!("Put the data in a struct and send it!");
}
