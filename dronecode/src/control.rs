use crate::control_trait::FSMControl;
use crate::fsm_manual_mode::FSMManual;
use crate::fsm_safe_mode::FSMSafe;
use crate::yaw_pitch_roll::YawPitchRoll;
use alloc::format;
use my_hdlc::command::{self, Command};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Led::Blue;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
use tudelft_quadrupel::nrf51_hal::uart;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

use my_hdlc::HdlcTransceiver;

const UART_BUF_SIZE: usize = 255usize;

pub fn main_loop() -> ! {
    set_tick_frequency(100);
    // send_bytes("Test\n".as_bytes());
    let mut last = Instant::now();
    let mut op_mode: &dyn FSMControl = &FSMManual;
    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();

    let mut command: Option<Command> = None;

    let mut receive_buffer = [0u8; my_hdlc::BUFFER_SIZE];
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

        // read input
        receive_bytes(&mut receive_buffer);

        transceiver.add_bytes(&receive_buffer);

        command = transceiver.read_structure::<Command>();

        // op_mode.run_control_loop(&command);

        if i % 100 == 0 {
            // send_bytes(format!("DTT: {:?}ms\n", dt.as_millis()).as_bytes());
            // send_bytes(
            //     format!(
            //         "MTR: {} {} {} {}\n",
            //         motors[0], motors[1], motors[2], motors[3]
            //     )
            //     .as_bytes(),
            // );
            // send_bytes(format!("YPR {} {} {}\n", ypr.yaw, ypr.pitch, ypr.roll).as_bytes());
            // send_bytes(format!("ACC {} {} {}\n", accel.x, accel.y, accel.z).as_bytes());
            // send_bytes(format!("BAT {bat}\n").as_bytes());
            // send_bytes(format!("BAR {pres}\n").as_bytes());
            // send_bytes("\n".as_bytes());
            // log data
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    unreachable!();
}
