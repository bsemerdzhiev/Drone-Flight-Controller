use my_hdlc::{command::DeviceCommand, HdlcTransceiver};
use std::sync::Arc;
use std::sync::Mutex;
use std::thread::sleep;
use std::time::Duration;
use tudelft_serial_upload::serial2::SerialPort;

use std::fmt;
use std::io::Write;
use std::os::unix::net::UnixStream;

use crate::runner_context::RunnerContext;

pub fn uplink_main_loop(ctx: &Arc<RunnerContext>) {
    let mut buf = Box::new([0u8; my_hdlc::BUFFER_SIZE]);
    loop {
        {
            let mut rcv = ctx.rcv_mut.lock().unwrap();
            let mut serial = ctx.serial_mut.lock().unwrap();

            if let Ok(num) = serial.read(&mut buf[0..rcv.bytes_to_read()]) {
                rcv.add_bytes(&buf[0..num]);
            }
        }
        {
            let mut rcv = ctx.rcv_mut.lock().unwrap();
            while let Some(msg) = rcv.read_structure::<my_hdlc::command::DeviceCommand>() {
                // ----------------
                match &msg {
                    DeviceCommand::Telemetry(telemetry) => {
                        let json = format!(
                            "{{\"Telemetry\": {}}}",
                            serde_json::to_string(telemetry).unwrap(),
                        );

                        {
                            let mut python_stream = ctx.python_stream_mut.lock().unwrap();

                            let _ = python_stream.write_all(json.as_bytes());
                            let _ = python_stream.write_all(b"\n");
                        }
                    }

                    _ => {}
                }
            }
        }
        sleep(Duration::from_micros(1));
    }
}
