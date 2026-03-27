use crate::downlink_comm::downlink_main_loop;
use crate::uplink_comm::uplink_main_loop;

use my_hdlc::command::DeviceCommand;
use my_hdlc::command::FSMState;
pub use my_hdlc::pc_command;
use my_hdlc::pc_command::ManualInput;
pub use my_hdlc::HdlcTransceiver;
use my_hdlc::STUFFED_MESSAGE_SIZE;

use tudelft_serial_upload::serial2::SerialPort;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

use std::env::args;
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

mod downlink_comm;
mod read_joystick;
mod read_keyboard;
mod uplink_comm;

fn main() {
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
    let mut serial_mut = Arc::new(Mutex::new(SerialPort::open(port, 115200).unwrap()));

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

    let mut python_stream_mut = Arc::new(Mutex::new(python_stream));
    let mut rcv_mut = Arc::new(Mutex::new(HdlcTransceiver::new()));

    let rcv_clone = Arc::clone(&rcv_mut);
    let serial_clone = Arc::clone(&serial_mut);
    let python_clone = Arc::clone(&python_stream_mut);
    let h1 = thread::spawn(move || {
        downlink_main_loop(&rcv_clone, &serial_clone, &python_clone);
    });

    let rcv_clone = Arc::clone(&rcv_mut);
    let serial_clone = Arc::clone(&serial_mut);
    let python_clone = Arc::clone(&python_stream_mut);
    let h2 = thread::spawn(move || {
        uplink_main_loop(&rcv_clone, &serial_clone, &python_clone);
    });

    h1.join().unwrap();
    h2.join().unwrap();
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
