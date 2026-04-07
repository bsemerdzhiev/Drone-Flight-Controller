use tudelft_quadrupel::{
    led::Led::{Green, Yellow},
    mutex::Mutex,
};

pub static BLE_BUFFER_SIZE: usize = 20usize;
pub static BLE_BUFFER: Mutex<([u8; BLE_BUFFER_SIZE], usize)> =
    Mutex::new(([0u8; BLE_BUFFER_SIZE], 0));

// #[link(name = "ble_app", kind = "static")]
extern "C" {
    // pub fn ble_init();
    pub fn ble_send(data: *const u8, length: u16);
}

// #[link(name = "ble_app", kind = "static")]
#[no_mangle]
pub extern "C" fn rust_ble_receive(data: *const u8, length: u16) {
    // Yellow.toggle();
    BLE_BUFFER.modify(|x| {
        let as_arr = unsafe { core::slice::from_raw_parts(data, length as usize) };

        for i in 0..length as usize {
            x.0[i] = as_arr[i];
        }
        x.1 = length as usize;
    });
}
