use crate::bluetooth::ble_connect;
use crate::downlink_comm::downlink_main_loop;
use crate::runner_context::RunnerContext;
use crate::uplink_comm::uplink_main_loop;
use crate::uplink_ui::rx_ui;

use my_hdlc::command::DeviceCommand;
use my_hdlc::command::FSMState;
pub use my_hdlc::pc_command;
use my_hdlc::pc_command::ManualInput;
pub use my_hdlc::HdlcTransceiver;
use my_hdlc::STUFFED_MESSAGE_SIZE;
use tokio;

use tudelft_serial_upload::serial2::SerialPort;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

use std::collections::VecDeque;
use std::env::args;
use std::error::Error;
use std::fs;
use std::io::Write;
use std::os::unix::net::UnixListener;
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::process::exit;
use std::process::Command;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use serde_json;

mod bluetooth;
mod downlink_comm;
mod read_joystick;
mod read_keyboard;
mod runner_context;
mod uplink_comm;
mod uplink_ui;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // get a filename from the command line. This filename will be uploaded to the drone
    // note that if no filename is given, the upload to the drone does not fail.
    // `upload_file_or_stop` will still try to detect the serial port on which the drone
    // is attached. This may be useful if you don't want to actually change the code on the
    // drone, but you do want to rerun your UI. In that case you simply don't provide any
    // command line parameter.
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);

    start_interface(&port);

    // open the serial port we got back from `upload_file_or_stop`. This is the same port
    // as the upload occurred on, so we know that we can communicate with the drone over
    // this port.
    let mut serial_mut = Mutex::new(SerialPort::open(port, 115200).unwrap());

    {
        let mut serial = serial_mut.lock().unwrap();
        serial.set_read_timeout(Duration::from_millis(400));
    }

    // --- Unix domain socket setup for Python GUI ---
    let socket_path = "/tmp/drone_telemetry.sock";
    let _ = fs::remove_file(socket_path);
    let listener = UnixListener::bind(socket_path).expect("Failed to bind UDS socket");
    println!("Waiting for Python GUI to connect…");
    let (mut python_stream, _) = listener
        .accept()
        .expect("Failed to accept Python connection");
    println!("Python GUI connected!");

    //-------------------------------------------------------------------------------------------
    let mut python_stream_mut = Mutex::new(python_stream);
    let mut rcv_mut = Mutex::new(HdlcTransceiver::new());
    let mut device_mut = Mutex::new(None);

    let mut keyboard_trim_mut = Mutex::new(ManualInput::default());
    let mut joystick_input_mut = Mutex::new(ManualInput::default());
    let mut joystick_disconnected_mut = Mutex::new(true);

    let mut is_wireless_mut = Mutex::new(false);
    let mut wireless_package_mut = Mutex::new(VecDeque::new());

    let mut current_state = Mutex::new(FSMState::SafeMode);
    //-------------------------------------------------------------------------------------------

    let mut ctx = Arc::new(RunnerContext {
        rcv_mut: rcv_mut,
        serial_mut: serial_mut,
        python_stream_mut: python_stream_mut,
        device_mut: device_mut,

        keyboard_trim_mut: keyboard_trim_mut,
        joystick_input_mut: joystick_input_mut,
        joystick_disconnected_mut: joystick_disconnected_mut,

        is_wireless_mut: is_wireless_mut,
        package_sender_mut: wireless_package_mut,
        current_state: current_state,
    });

    let ctx_clone = Arc::clone(&ctx);

    let h1 = thread::spawn(move || {
        downlink_main_loop(&ctx_clone);
    });

    let ctx_clone = Arc::clone(&ctx);
    let h2 = thread::spawn(move || {
        uplink_main_loop(&ctx_clone);
    });

    let ctx_clone = Arc::clone(&ctx);
    let h2 = thread::spawn(move || {
        rx_ui(&ctx_clone);
    });

    let ctx_clone = Arc::clone(&ctx);
    ble_connect(&ctx_clone).await?;
    h1.join().unwrap();
    h2.join().unwrap();

    Ok(())
}

#[allow(unused)]
fn start_interface(port: &PathBuf) {
    let project_root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap();

    let mut cmd = Command::new("python");
    cmd.arg("ui/main.py")
        .arg(port.to_str().unwrap())
        .current_dir(&project_root)
        .spawn()
        .expect("cannot open UI");
}
