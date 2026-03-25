use core::time::Duration;

use alloc::boxed::Box;
use alloc::format;
use nrf51_pac::RADIO;

use crate::states::fsm_base_class::FSMControl;
use crate::states::manual_mode::FSMManual;
use crate::states::safe_mode::FSMSafe;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::states::state_structures::state_context::StateContext;
use crate::telemetry_read::TelemetryRead;
use crate::wireless_setup::{self, *};

use my_hdlc::command::{self, DeviceCommand, DroneInfo, FSMState, WirelessOptions};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::{HdlcTransceiver, STUFFED_MESSAGE_SIZE};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::{Blue, Green};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{enable_dmp, read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

// -------------------------------------------------------------------------

const UART_BUF_SIZE: usize = my_hdlc::BUFFER_SIZE;

// -------------------------------------------------------------------------

// in ms
const WATCHDOG_TIMER_FOR_PANICKING: Duration = Duration::from_millis(300);

const SHOULD_CHECK_BATTERY_LEVEL: bool = true;
const MIN_BAT_LEVEL: u16 = 1050;

// -------------------------------------------------------------------------

pub fn main_loop() -> ! {
    // processor tick frequency
    set_tick_frequency(100);
    // -------------------------------------------------------------------------

    // buffer for receiving bytes from PC
    let mut uart_buf = [0u8; UART_BUF_SIZE];

    // -------------------------------------------------------------------------

    // current state in FSM
    let mut current_state: Box<dyn FSMControl> = Box::new(FSMSafe {});

    // -------------------------------------------------------------------------

    // fields for the context
    let mut transceiver: HdlcTransceiver = HdlcTransceiver::new();
    let mut received_manual_input: Option<ManualInput> = None;
    let mut calibration_state: CalibrationState = CalibrationState::new();
    let mut wireless_toggle = false; 
    let mut wireless_option = WirelessOptions::PCSide;

    let mut ctx = StateContext {
        calibration_state: &mut calibration_state,
        trv: &mut transceiver,
        input_from_controller: &mut received_manual_input,
        wireless_toggle,
        wireless_option,
    };

    // -------------------------------------------------------------------------

    // used for determining whether we should panic
    let mut time_for_last_received_message: Instant = Instant::now();

    // used to determine whether battery voltage is too low
    let mut battery_panic = false;

    // radio object
    let radio = wireless_setup::radio_init();

    radio_start_rx(&radio);
    // -------------------------------------------------------------------------
    for i in 0.. {
        let _ = Blue.toggle();

        // -------------------------------------------------------------------------
        // Check battery level and switch to panic
        let bat_level = read_battery();
        if SHOULD_CHECK_BATTERY_LEVEL && bat_level < MIN_BAT_LEVEL {
            current_state = current_state.step(command::FSMState::PanicMode, &mut ctx);
            battery_panic = true;
        } else if battery_panic && bat_level >= MIN_BAT_LEVEL {
            current_state = current_state.step(command::FSMState::SafeMode, &mut ctx);
            battery_panic = false;
        }
        // -------------------------------------------------------------------------

        if ctx.wireless_toggle {
            match ctx.wireless_option {
                WirelessOptions::PCSide => {
                    // Read Uart Buff
                    let num_received = receive_bytes(&mut uart_buf[0..ctx.trv.remaining_bytes]);

                    if num_received != 0usize {
                        //read the sent bytes
                        ctx.trv.add_bytes(&uart_buf[..num_received]);

                        //try to deserialize the command
                        let deserialized_command = ctx.trv.read_structure::<DeviceCommand>();

                        // if there is a command
                        if let Some(command) = deserialized_command {
                            match command {
                                DeviceCommand::ChangeMode(new_mode) => {
                                    current_state = current_state.step(new_mode, &mut ctx);
                                    send_ack(&mut ctx.trv, ctx.wireless_option, &radio);
                                }               
                                DeviceCommand::ManualInput(manual_input) => {
                                    *ctx.input_from_controller = Some(manual_input);
                                }
                                _ => continue,
                            }
                            time_for_last_received_message = Instant::now();
                        }
                    }
                }

                WirelessOptions::DroneSide => {
                    if let Some(command) = radio_poll_rx(&radio, &mut ctx.trv) {
                        match command {
                            DeviceCommand::ChangeMode(new_mode) => {
                                current_state = current_state.step(new_mode, &mut ctx);
                                send_ack(&mut ctx.trv, ctx.wireless_option, &radio);
                            }               
                            DeviceCommand::ManualInput(manual_input) => {
                                *ctx.input_from_controller = Some(manual_input);
                            }
                            _ => continue,
                        }
                        time_for_last_received_message = Instant::now();
                    }
                    radio_start_rx(&radio);
                }
            }
        } else {
            // Read Uart Buff
            let num_received = receive_bytes(&mut uart_buf[0..ctx.trv.remaining_bytes]);

            if num_received != 0usize {
                //read the sent bytes
                ctx.trv.add_bytes(&uart_buf[..num_received]);

                //try to deserialize the command
                let deserialized_command = ctx.trv.read_structure::<DeviceCommand>();

                // if there is a command
                if let Some(command) = deserialized_command {
                    match command {
                        DeviceCommand::ChangeMode(new_mode) => {
                            current_state = current_state.step(new_mode, &mut ctx);
                            send_ack(&mut ctx.trv, ctx.wireless_option, &radio);
                        }               
                        DeviceCommand::ManualInput(manual_input) => {
                            *ctx.input_from_controller = Some(manual_input);
                        }
                        _ => continue,
                    }
                    time_for_last_received_message = Instant::now();
                }
            }
        }
        if Instant::now().duration_since(time_for_last_received_message)
            >= WATCHDOG_TIMER_FOR_PANICKING
        {
            if ctx.wireless_option == WirelessOptions::PCSide {
                current_state = current_state.step(command::FSMState::PanicMode, &mut ctx);
            }
        }

        // run the loop of the state
        current_state = current_state.run_state_loop(&mut ctx);
        if i % 10 == 0 {
            send_drone_data(&mut ctx.trv, current_state.get_state(),ctx.wireless_option, &radio);
            Green.off();
        }

        // -------------------------------------------------------------------------
        // send information about the drone state to PC
        let to_write = ctx
            .trv
            .write_structure(&DeviceCommand::DroneInfo(DroneInfo::new(
                current_state.get_state(),
                read_battery(),
            )));

        if ctx.wireless_toggle {
            wireless_setup::radio_send(&radio, &to_write.0[0..to_write.1]);
        } else {
            send_bytes(&to_write.0[0..to_write.1]);
        }
        
        wait_for_next_tick();
    }
    unreachable!();
}

/*
* Sends data to the drone
*/
fn send_drone_data(transceiver: &mut HdlcTransceiver, current_state: FSMState, wireless_mode: WirelessOptions, radio: &RADIO) {
    let msg = transceiver.write_structure(&DeviceCommand::DroneInfo(DroneInfo::new(
        current_state,
        read_battery(),
    )));
    Green.on();

    match wireless_mode {
        WirelessOptions::PCSide => {
            let _ = send_bytes(&msg.0[0..msg.1]);
        }
        WirelessOptions::DroneSide => wireless_setup::radio_send(radio, &msg.0[0..msg.1]),
    }
}

/*
* Sends ACKs to the drone after a state change
*/
fn send_ack(transceiver: &mut HdlcTransceiver, wireless_mode: WirelessOptions, radio: &RADIO) {
    let ack_cmd = DeviceCommand::Ack;
    let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = transceiver.write_structure(&ack_cmd);

    match wireless_mode {
        WirelessOptions::PCSide => {
            let _ = send_bytes(&msg.0[0..msg.1]);
        }
        WirelessOptions::DroneSide => wireless_setup::radio_send(radio, &msg.0[0..msg.1]),
    }
}
