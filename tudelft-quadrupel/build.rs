fn main() {
    // println!("cargo:warning=cwd: {:?}", std::env::current_dir().unwrap());
    let status = std::process::Command::new("make")
        .arg("static_lib")
        .current_dir("../ble_setup")
        .status()
        .expect("failed to run make");

    assert!(status.success(), "make failed");

    let dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();

    println!("cargo:rustc-link-search=native={}/../ble_setup/_build", dir);
    println!("cargo:rustc-link-lib=static=ble_app");
    println!("cargo:rerun-if-changed=../ble_setup/ble_app_uart/main.c");
}
