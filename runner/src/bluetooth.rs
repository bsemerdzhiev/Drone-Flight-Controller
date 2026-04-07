use std::error::Error;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use std::io::Write;
use std::os::unix::net::UnixStream;

use btleplug::api::bleuuid::uuid_from_u32;
use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter, WriteType};
use btleplug::platform::{Adapter, Manager, Peripheral};
use futures::StreamExt;
use my_hdlc::STUFFED_MESSAGE_SIZE;
use tokio::{self, time};
use uuid::{uuid, Uuid};

use crate::runner_context::RunnerContext;

const RX_CHAR_UUID: Uuid = uuid!("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const TX_CHAR_UUID: Uuid = uuid!("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

pub async fn ble_connect(ctx: &Arc<RunnerContext>) -> Result<(), Box<dyn Error>> {
    // println!("Start");
    let manager = Manager::new().await.unwrap();

    let adapters = manager.adapters().await?;
    let central = adapters.into_iter().nth(0).unwrap();

    // println!("Searching for devices");
    central.start_scan(ScanFilter::default()).await?;

    time::sleep(Duration::from_secs(2)).await;

    let drone = find_drone(&central).await.unwrap();

    drone.connect().await?;

    println!("Connected to drone");

    drone.discover_services().await?;

    let chars = drone.characteristics();
    let rx_char = chars.iter().find(|c| c.uuid == RX_CHAR_UUID).unwrap();
    let tx_char = chars.iter().find(|c| c.uuid == TX_CHAR_UUID).unwrap();

    drone.subscribe(tx_char).await?;
    let mut notif_stream = drone.notifications().await?;

    let drone_clone = drone.clone();
    let rx_char_clone = rx_char.clone();

    let ctx_clone = Arc::clone(ctx);
    tokio::spawn(async move {
        while let Some(data) = notif_stream.next().await {
            let wireless_mode: bool = ctx_clone.with_is_wireless(|s| *s);

            if wireless_mode {
                let mut rcv = ctx_clone.rcv_mut.lock().unwrap();

                for cur_byte in data.value {
                    rcv.add_byte(cur_byte);
                }
            }
        }
    });

    let ctx_clone = Arc::clone(ctx);
    tokio::spawn(async move {
        let mut send_buffer: [u8; STUFFED_MESSAGE_SIZE] = [0u8; STUFFED_MESSAGE_SIZE];

        loop {
            let packet_opt: Option<Vec<u8>> = ctx_clone.with_package_sender(|s| s.pop_front());

            let wireless_mode: bool = ctx_clone.with_is_wireless(|s| *s);

            if let Some(packet) = packet_opt {
                if wireless_mode {
                    if packet.len() <= 20 {
                        drone
                            .write(&rx_char_clone, &packet, WriteType::WithoutResponse)
                            .await
                            .unwrap();
                    } else {
                        println!("Packet size too large over bluetooth\r");
                    }

                    tokio::time::sleep(Duration::from_millis(100)).await;
                } else {
                    let packet_len = packet.len();

                    for i in 0..packet_len {
                        send_buffer[i] = packet[i];
                    }

                    ctx_clone.with_serial(|x| x.write(&send_buffer[0..packet_len]));

                    tokio::time::sleep(Duration::from_millis(50)).await;
                }
            }

            // ctx_clone.with_wireless_package(|s| *s = vec![]);
        }
    });

    // sends data to the ui
    let ctx_clone = Arc::clone(ctx);
    tokio::spawn(async move {
        loop {
            if let Ok(rssi) = drone_clone.read_rssi().await {
                let json = serde_json::to_string(&serde_json::json!({
                    "BLEInfo": {
                        "rssi" : rssi
                    }
                }))
                .unwrap();

                let mut python_stream = ctx_clone.python_stream_mut.lock().unwrap();

                let _ = python_stream.write_all(json.as_bytes());
                let _ = python_stream.write_all(b"\n");
            }
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
    });

    Ok(())
}

async fn find_drone(central: &Adapter) -> Option<Peripheral> {
    for p in central.peripherals().await.unwrap() {
        if p.properties()
            .await
            .unwrap()
            .unwrap()
            .local_name
            .iter()
            .any(|name| name.contains("Nordic_UART"))
        {
            // println!("{:?}", p);
            return Some(p);
        }
    }
    None
}
