fn main() {
    let sdk = "nrf_sdk";

    let mut build = cc::Build::new();
    build
        .compiler("arm-none-eabi-gcc")
        .target("thumbv6m-none-eabi")
        .flag("-DSOFTDEVICE_PRESENT")
        .flag("-DNRF51")
        .flag("-DS110")
        .flag("-DBLE_STACK_SUPPORT_REQD")
        .flag("-DSWI_DISABLE0")
        .flag("-mcpu=cortex-m0")
        .flag("-mthumb")
        .flag("-mabi=aapcs")
        .flag("--std=gnu99")
        .flag("-O3")
        .flag("-mfloat-abi=soft")
        .flag("-ffunction-sections")
        .flag("-fdata-sections")
        .flag("-fno-strict-aliasing")
        .flag("-fno-builtin")
        .flag("--short-enums")
        // C source files
        .file("src/ble_app.c")
        .file(format!("{}/components/libraries/button/app_button.c", sdk))
        .file(format!("{}/components/libraries/util/app_error.c", sdk))
        .file(format!("{}/components/libraries/fifo/app_fifo.c", sdk))
        .file(format!("{}/components/libraries/timer/app_timer.c", sdk))
        .file(format!("{}/components/libraries/trace/app_trace.c", sdk))
        .file(format!("{}/components/libraries/util/nrf_assert.c", sdk))
        .file(format!("{}/components/drivers_nrf/delay/nrf_delay.c", sdk))
        .file(format!(
            "{}/components/drivers_nrf/common/nrf_drv_common.c",
            sdk
        ))
        // .file(format!(
        // "{}/components/drivers_nrf/gpiote/nrf_drv_gpiote.c",
        // sdk
        // ))
        // .file(format!(
        // "{}/components/drivers_nrf/uart/nrf_drv_uart.c",
        // sdk
        // ))
        .file(format!(
            "{}/components/drivers_nrf/pstorage/pstorage.c",
            sdk
        ))
        .file(format!("{}/components/ble/common/ble_advdata.c", sdk))
        .file(format!(
            "{}/components/ble/ble_advertising/ble_advertising.c",
            sdk
        ))
        .file(format!("{}/components/ble/common/ble_conn_params.c", sdk))
        .file(format!(
            "{}/components/ble/ble_services/ble_nus/ble_nus.c",
            sdk
        ))
        .file(format!("{}/components/ble/common/ble_srv_common.c", sdk))
        .file(format!("{}/components/toolchain/system_nrf51.c", sdk))
        .file(format!(
            "{}/components/softdevice/common/softdevice_handler/softdevice_handler.c",
            sdk
        ))
        // includes
        .include("main_include")
        .include(format!("{}/components/drivers_nrf/config", sdk))
        .include(format!("{}/components/libraries/fifo", sdk))
        .include(format!("{}/components/drivers_nrf/delay", sdk))
        .include(format!("{}/components/libraries/util", sdk))
        // .include(format!("{}/components/drivers_nrf/uart", sdk))
        .include(format!("{}/components/ble/common", sdk))
        .include(format!("{}/components/drivers_nrf/pstorage", sdk))
        .include(format!(
            "{}/examples/ble_peripheral/ble_app_uart/config",
            sdk
        ))
        // .include(format!("{}/components/libraries/uart", sdk))
        .include(format!("{}/components/device", sdk))
        .include(format!("{}/components/libraries/button", sdk))
        .include(format!("{}/components/libraries/timer", sdk))
        .include(format!("{}/components/softdevice/s110/headers", sdk))
        .include(format!("{}/components/drivers_nrf/gpiote", sdk))
        .include(format!("{}/components/ble/ble_services/ble_nus", sdk))
        .include(format!("{}/components/drivers_nrf/hal", sdk))
        .include(format!("{}/components/toolchain/gcc", sdk))
        .include(format!("{}/components/toolchain", sdk))
        .include(format!("{}/components/drivers_nrf/common", sdk))
        .include(format!("{}/components/ble/ble_advertising", sdk))
        .include(format!("{}/components/libraries/trace", sdk))
        .include(format!(
            "{}/components/softdevice/common/softdevice_handler",
            sdk
        ));

    // let objects = build.compile_intermediates();

    // for obj in objects {
    // println!("cargo:rustc-link-arg={}", obj.display());
    // }
    build.compile("ble_app");

    println!("cargo:rerun-if-changed=src/ble_app.c");
    // println!("cargo:rustc-link-lib=static=ble_app");
}
