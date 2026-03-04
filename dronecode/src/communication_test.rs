pub use my_hdlc::HdlcTransceiver;
use my_hdlc::{
    command::{DeviceCommand, FSMState},
    STUFFED_MESSAGE_SIZE,
};
use tudelft_quadrupel::uart;

pub fn send_and_receive() -> ! {
    let mut rscv = HdlcTransceiver::new();

    while (true) {
        // let msg_to_rcv: Option<DeviceCommand> = rscv.read_structure::<DeviceCommand>();

        // let cmd: DeviceCommand = DeviceCommand::new(
        //     my_hdlc::DeviceCommand::CommandType::ChangeMode,
        //     Some(FSMState::SafeMode),
        // );
        let cmd: DeviceCommand = DeviceCommand::ChangeMode(FSMState::SafeMode);

        let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = rscv.write_structure(&cmd);

        uart::send_bytes(&msg.0[0..msg.1]);
    }
    unreachable!();
}
