use my_hdlc::{command::DeviceCommand, HdlcTransceiver};
use std::sync::Arc;
use std::sync::Mutex;
use std::thread::sleep;
use std::time::Duration;
use tudelft_serial_upload::serial2::SerialPort;

use std::fmt;
use std::io::Write;
use std::os::unix::net::UnixStream;

pub fn uplink_main_loop(
    rcv_mut: &Arc<Mutex<HdlcTransceiver>>,
    serial_mut: &Arc<Mutex<SerialPort>>,
    python_stream_mut: &Arc<Mutex<UnixStream>>,
) {
    let mut buf = [0u8; my_hdlc::BUFFER_SIZE];
    loop {
        {
            let mut rcv = rcv_mut.lock().unwrap();
            let mut serial = serial_mut.lock().unwrap();

            if let Ok(num) = serial.read(&mut buf[0..rcv.remaining_bytes]) {
                rcv.add_bytes(&buf[0..num]);
            }
        }

        let mut msg = None;
        {
            let mut rcv = rcv_mut.lock().unwrap();
            msg = rcv.read_structure::<my_hdlc::command::DeviceCommand>();
        }

        if let Some(msg) = msg {
            // ----------------
            match &msg {
                DeviceCommand::Telemetry(telemetry) => {
                    let json = format!(
                        "{{\"Telemetry\": {}}}",
                        serde_json::to_string(telemetry).unwrap(),
                    );

                    {
                        let mut python_stream = python_stream_mut.lock().unwrap();

                        let _ = python_stream.write_all(json.as_bytes());
                        let _ = python_stream.write_all(b"\n");
                    }
                }

                _ => {}
            }
        }
        sleep(Duration::from_micros(10));
    }
}
