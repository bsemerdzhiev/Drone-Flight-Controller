fn main() {
    eprintln!("DRONECODE BUILD.RS IS RUNNING");

    println!("cargo:rustc-link-lib=static=ble_app");
}
