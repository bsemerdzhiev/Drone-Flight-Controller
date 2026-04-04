use tudelft_quadrupel::led::Led::{Green, Yellow};

#[link(name = "ble_app", kind = "static")]
extern "C" {
    // pub fn ble_init();
    pub fn ble_send(data: *const u8, length: u16);
}

#[link(name = "ble_app", kind = "static")]
#[no_mangle]
pub extern "C" fn rust_ble_receive(data: *const u8, length: u16) {
    let as_arr = unsafe { core::slice::from_raw_parts(data, length as usize) };
    if (as_arr[0] == '1' as u8) {
        Yellow.toggle();
    }
}
