use crate::runner_context::RunnerContext;
use std::io::{BufRead, BufReader};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use my_hdlc::command::DeviceCommand;
use my_hdlc::pc_command::{ManualDroneTrimsEnums, PIDValues};
use serde::{Deserialize, Serialize};
use serde_json::value;

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Default)]
pub struct PidTerms {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

impl From<PidTerms> for PIDValues {
    fn from(value: PidTerms) -> Self {
        Self {
            p_value: value.p,
            i_value: value.i,
            d_value: value.d,
        }
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Default)]
pub struct PidTrims {
    pub yaw: PidTerms,
    pub pitch: PidTerms,
    pub roll: PidTerms,
    pub lift: PidTerms,
}

pub fn rx_ui(ctx: &Arc<RunnerContext>) {
    loop {
        let stream = { ctx.with_python_stream(|x| x.try_clone().unwrap()) };

        let mut reader = BufReader::new(stream);

        for line in reader.lines() {
            let line = line.unwrap();

            let trims_res = serde_json::from_str::<PidTrims>(&line);

            if trims_res.is_err() {
                continue;
            }

            let trims = trims_res.unwrap();

            let wireless_mode: bool = ctx.with_is_wireless(|s| *s);

            if (!wireless_mode) {
                let mut rcv = ctx.rcv_mut.lock().unwrap();
                ctx.with_package_sender(|s| {
                    let send_buffer =
                        rcv.write_structure::<DeviceCommand>(&DeviceCommand::ManualDroneTrims(
                            ManualDroneTrimsEnums::Lift(PIDValues::from(trims.lift)),
                        ));
                    s.push_back(send_buffer.0[0..send_buffer.1].to_vec());

                    let send_buffer =
                        rcv.write_structure::<DeviceCommand>(&DeviceCommand::ManualDroneTrims(
                            ManualDroneTrimsEnums::Yaw(PIDValues::from(trims.yaw)),
                        ));
                    s.push_back(send_buffer.0[0..send_buffer.1].to_vec());

                    let send_buffer =
                        rcv.write_structure::<DeviceCommand>(&DeviceCommand::ManualDroneTrims(
                            ManualDroneTrimsEnums::Pitch(PIDValues::from(trims.pitch)),
                        ));
                    s.push_back(send_buffer.0[0..send_buffer.1].to_vec());

                    let send_buffer =
                        rcv.write_structure::<DeviceCommand>(&DeviceCommand::ManualDroneTrims(
                            ManualDroneTrimsEnums::Roll(PIDValues::from(trims.roll)),
                        ));
                    s.push_back(send_buffer.0[0..send_buffer.1].to_vec());
                });
            }

            // println!("{}\n", line);
        }

        thread::sleep(Duration::from_millis(500));
    }
}
