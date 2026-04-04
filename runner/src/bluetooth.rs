use std::error::Error;
use std::time::Duration;

use btleplug::api::bleuuid::uuid_from_u32;
use btleplug::api::{Central, Manager as _, Peripheral as _, ScanFilter};
use btleplug::platform::{Adapter, Manager, Peripheral};
use tokio::{self, time};
use uuid::{uuid, Uuid};

const RX_CHAR_UUID: Uuid = uuid!("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const TX_CHAR_UUID: Uuid = uuid!("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

pub async fn ble_connect() -> Result<(), Box<dyn Error>> {
    // println!("Start");
    let manager = Manager::new().await.unwrap();

    let adapters = manager.adapters().await?;
    let central = adapters.into_iter().nth(0).unwrap();

    // println!("Searching for devices");
    central.start_scan(ScanFilter::default()).await?;

    time::sleep(Duration::from_secs(4)).await;

    let drone = find_drone(&central).await.unwrap();

    drone.connect().await?;

    time::sleep(Duration::from_secs(15)).await;

    println!("Connected to drone");

    drone.discover_services().await?;

    let chars = drone.characteristics();
    let rx_char = chars.iter().find(|c| c.uuid == RX_CHAR_UUID).unwrap();
    let tx_char = chars.iter().find(|c| c.uuid == TX_CHAR_UUID).unwrap();

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
