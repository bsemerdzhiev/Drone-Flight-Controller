#[link(name = "ble_app", kind = "static")]
extern "C" {
    // pub fn ble_init();
    pub fn ble_send(data: *const u8, length: u16);
}

#[link(name = "ble_app", kind = "static")]
#[no_mangle]
pub extern "C" fn rust_ble_receive(data: *const u8, length: u16) {}
