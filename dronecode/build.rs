fn main() {
    // Point at where ble-c-extern's build.rs puts the archive
    // This path is stable as long as the ble-c-extern package version doesn't change
    println!("cargo:rerun-if-changed=../ble-c-extern/src/ble_app.c");

    // Let Cargo find the archive via the dependency's OUT_DIR
    // We can't read OUT_DIR of another crate directly, so we just
    // re-emit the link against the search path Cargo already sets up
    println!("cargo:rustc-link-lib=static=ble_app");
}
