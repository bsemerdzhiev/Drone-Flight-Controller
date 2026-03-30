use core::time::Duration;

use crate::filters::dmp_readings::DmpReadings;
use crate::filters::kalman_filter::KalmanFilter;
use crate::filters::pressure_filter::PressureSensor;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::telemetry_read::TelemetryRead;

use alloc::boxed::Box;
use alloc::format;
use my_hdlc::telemetry_data::*;

use tudelft_quadrupel::flash::flash_write_bytes;

use crate::states::fsm_base_class::FSMControl;
use crate::states::manual_mode::FSMManual;
use crate::states::safe_mode::FSMSafe;
use crate::states::state_structures::state_context::StateContext;

use my_hdlc::command::{self, DeviceCommand, DroneInfo, FSMState};
use my_hdlc::pc_command::ManualInput;
use my_hdlc::{HdlcTransceiver, STUFFED_MESSAGE_SIZE};
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Led::{Blue, Green, Red, Yellow};
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{enable_dmp, read_dmp_bytes, read_raw};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{receive_bytes, send_bytes};

// -------------------------------------------------------------------------

const UART_BUF_SIZE: usize = my_hdlc::BUFFER_SIZE;

// -------------------------------------------------------------------------

// in ms
const WATCHDOG_TIMER_FOR_PANICKING: Duration = Duration::from_millis(1500);
const DRONE_STATE_TIMER: Duration = Duration::from_millis(1);

const SHOULD_CHECK_BATTERY_LEVEL: bool = false;
const MIN_BAT_LEVEL: u16 = 1050;

// -------------------------------------------------------------------------

pub fn main_loop() -> ! {
    // processor tick frequency
    set_tick_frequency(100);
    // -------------------------------------------------------------------------

    // buffer for receiving bytes from PC
    let mut uart_buf = Box::new([0u8; UART_BUF_SIZE]);

    // -------------------------------------------------------------------------

    // current state in FSM
    let mut current_state: Box<dyn FSMControl> = Box::new(FSMSafe {});

    // -------------------------------------------------------------------------

    // Time data for telemetry data
    let current_time = Instant::now();
    // -------------------------------------------------------------------------

    // fields for the context
    let mut transceiver: Box<HdlcTransceiver> = Box::new(HdlcTransceiver::new());

    let mut received_manual_input: Option<ManualInput> = None;
    let mut calibration_state: CalibrationState = CalibrationState::new();
    let mut flash_head = 0usize;
    let mut flash_tail = 0usize;

    let mut pressure_sensor_filter = PressureSensor::new();
    let mut position_kalman = KalmanFilter::new((
        calibration_state.accelerometer_offset,
        calibration_state.gyro_offset,
    ));
    let mut dmp_sampler = DmpReadings::new(calibration_state.ypr_offset);

    let mut ctx = StateContext {
        kalman_position: position_kalman,
        calibration_state: &mut calibration_state,
        pressure_sensor_filter: &mut pressure_sensor_filter,
        dmp_filter: dmp_sampler,

        trv: &mut transceiver,
        input_from_controller: &mut received_manual_input,
        flash_head: &mut flash_head,
        flash_tail: &mut flash_tail,
    };

    // -------------------------------------------------------------------------

    // used for determining whether we should panic
    let mut time_for_last_received_message: Instant = Instant::now();
    let mut last_send_message: Instant = Instant::now();

    let cal = CalibrationState::new();

    // used to determine whether battery voltage is too low
    let mut battery_panic = false;
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

        // Read Uart Buff
        let num_received = receive_bytes(&mut uart_buf[0..ctx.trv.bytes_to_read()]);

        if num_received != 0usize {
            //read the sent bytes
            ctx.trv.add_bytes(&uart_buf[..num_received]);

            //try to deserialize the command
            let deserialized_command = ctx.trv.read_structure::<DeviceCommand>();

            // if there is a command
            if let Some(command) = deserialized_command {
                // Red.toggle();
                match command {
                    DeviceCommand::ChangeMode(new_mode) => {
                        current_state = current_state.step(new_mode, &mut ctx);
                        send_ack(&mut ctx.trv);
                    }
                    DeviceCommand::ManualInput(manual_input) => {
                        *ctx.input_from_controller = Some(manual_input);
                    }
                    _ => {}
                }
                time_for_last_received_message = Instant::now();
            }
        }

        let now = Instant::now();
        if now.duration_since(time_for_last_received_message) >= WATCHDOG_TIMER_FOR_PANICKING {
            current_state = current_state.step(command::FSMState::PanicMode, &mut ctx);
        }

        let dt = now.duration_since(current_time);

        // update filter readings
        ctx.kalman_position.append_new_reading();
        ctx.pressure_sensor_filter
            .update_readings(&mut ctx.kalman_position);

        // run the loop of the state
        current_state = current_state.run_state_loop(&mut ctx);

        if now.duration_since(last_send_message) >= DRONE_STATE_TIMER {
            last_send_message = now;

            send_drone_data(current_state.get_state(), dt, &mut ctx);
        }

        put_telemetry_data_on_flash(&mut ctx, dt, current_state.get_state());

        // -------------------------------------------------------------------------

        wait_for_next_tick();
    }
    unreachable!();
}

/*
* Sends data to the drone
*/
fn send_drone_data(curent_state: FSMState, dt: Duration, ctx: &mut StateContext) {
    Green.on();

    let mut msg =
        <TelemetryData as TelemetryRead>::read_general_data(ctx, dt, curent_state, false, true);
    send_bytes(&msg.0[0..msg.1]);

    msg = <TelemetryData as TelemetryRead>::read_motor_data(ctx, false, true);
    send_bytes(&msg.0[0..msg.1]);

    msg = <TelemetryData as TelemetryRead>::read_position_data(ctx, false, true);
    send_bytes(&msg.0[0..msg.1]);

    msg = <TelemetryData as TelemetryRead>::read_pressure_data(ctx, false, true);
    send_bytes(&msg.0[0..msg.1]);

    msg = <TelemetryData as TelemetryRead>::read_raw_data(ctx, false, true);
    send_bytes(&msg.0[0..msg.1]);

    Green.off();
}

/*
* Sends ACKs to the drone after a state change
*/
fn send_ack(transceiver: &mut HdlcTransceiver) {
    let ack_cmd = DeviceCommand::Ack;
    let msg: ([u8; STUFFED_MESSAGE_SIZE], usize) = transceiver.write_structure(&ack_cmd);
    send_bytes(&msg.0[0..msg.1]);
}

fn put_telemetry_data_on_flash(ctx: &mut StateContext, dt: Duration, curent_state: FSMState) {
    if *ctx.flash_tail + (STUFFED_MESSAGE_SIZE * 5) > 0x01FFFF {
        return;
    }

    Yellow.on();

    let mut msg =
        <TelemetryData as TelemetryRead>::read_general_data(ctx, dt, curent_state, true, false);
    _ = flash_write_bytes(*ctx.flash_tail as u32, &msg.0[0..msg.1]);
    *ctx.flash_tail += STUFFED_MESSAGE_SIZE;

    msg = <TelemetryData as TelemetryRead>::read_motor_data(ctx, true, false);
    _ = flash_write_bytes(*ctx.flash_tail as u32, &msg.0[0..msg.1]);
    *ctx.flash_tail += STUFFED_MESSAGE_SIZE;

    msg = <TelemetryData as TelemetryRead>::read_position_data(ctx, true, false);
    _ = flash_write_bytes(*ctx.flash_tail as u32, &msg.0[0..msg.1]);
    *ctx.flash_tail += STUFFED_MESSAGE_SIZE;

    msg = <TelemetryData as TelemetryRead>::read_pressure_data(ctx, true, false);
    _ = flash_write_bytes(*ctx.flash_tail as u32, &msg.0[0..msg.1]);
    *ctx.flash_tail += STUFFED_MESSAGE_SIZE;

    msg = <TelemetryData as TelemetryRead>::read_raw_data(ctx, true, false);
    _ = flash_write_bytes(*ctx.flash_tail as u32, &msg.0[0..msg.1]);
    *ctx.flash_tail += STUFFED_MESSAGE_SIZE;

    Yellow.off();
}
