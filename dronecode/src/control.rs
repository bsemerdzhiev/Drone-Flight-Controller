use crate::control_trait::FSMControl;
use crate::fsm_safe_mode::FSMSafe;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use my_hdlc::HdlcTransceiver;

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

        let motors = get_motors();
        let quaternion = block!(read_dmp_bytes()).unwrap();
        let ypr = YawPitchRoll::from(quaternion);
        let (accel, _) = read_raw().unwrap();
        let bat = read_battery();
        let pres = read_pressure();

        op_mode.run_control_loop(&mut transceiver);

        if i % 100 == 0 {
            // log data
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}
