fn main() {
    let dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search=native={}/..", dir);
    println!("cargo:rustc-link-lib=static=ble_app");
}
