use crate::control_trait::FSMControl;
use crate::fsm_safe_mode::FSMSafe;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use my_hdlc::command::{Command, CommandType, FSMState};
use my_hdlc::HdlcTransceiver;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};
const UART_BUF_SIZE: usize = 255usize;

pub fn main_loop() -> ! {
    set_tick_frequency(100);
    let mut last = Instant::now();
    let mut op_mode: &dyn FSMControl = &FSMSafe;
    let mut uart_buf = [0u8; UART_BUF_SIZE];
    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();
    for i in 0.. {
        let _ = Blue.toggle();
        let now = Instant::now();
        let dt = now.duration_since(last);
        last = now;

        // Read Uart Buff
        let num_received = receive_bytes(&mut uart_buf);
        if num_received != 0usize {
            transceiver.add_bytes(&uart_buf[..num_received]);
            let deserialized_command =
                transceiver.read_structure::<my_hdlc::command::CommandType>();
            if let Some(command) = deserialized_command {
                run_command(command);
            }
        }

        // control_loop(op_mode);
        op_mode.run_control_loop();
        if i % 100 == 0 {
            // send_drone_data();
        }

        // Control Loop:
        // Read Command and Execute, (if available).
        // Run the current mode's control loop
        // send data if i%100 == 0
        // wait until the timer interrupt goes off again
        // based on the frequency set above

        wait_for_next_tick();
    }
    unreachable!();
}

fn run_command(command: CommandType) {
    todo!("Execute Commands!");
}

fn send_drone_data() {
    let motors = get_motors();
    let quaternion = block!(read_dmp_bytes()).unwrap();
    let ypr = YawPitchRoll::from(quaternion);
    let (accel, _) = read_raw().unwrap();
    let bat = read_battery();
    let pres = read_pressure();
    todo!("Put the data in a struct and send it!");
}
