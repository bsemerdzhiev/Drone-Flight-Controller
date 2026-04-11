use core::time::Duration;

use crate::filters::dmp_readings::DmpReadings;
use crate::filters::kalman_filter::KalmanFilter;
use crate::filters::pressure_filter::PressureSensor;
use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::calibration_state::CalibrationState;
use crate::telemetry_read::{ReturnType, TelemetryRead};
use crate::util::ble_communication::{BLE_BUFFER, BLE_BUFFER_SIZE};
use crate::util::pid_controller::{ControllerValues, PIDController};
use crate::util::yaw_pitch_roll::YawPitchRoll;

use alloc::boxed::Box;
use alloc::format;
use fixed::types::{I16F16, I26F6, I2F30, I4F28, I8F24};
use my_hdlc::telemetry_data::*;

use template_project::util::ble_communication::{ble_send, rust_ble_receive};
use tudelft_quadrupel::flash::flash_write_bytes;

use crate::states::fsm_base_class::FSMControl;
use crate::states::manual_mode::FSMManual;
use crate::states::safe_mode::FSMSafe;
use crate::states::state_structures::state_context::{
    PIDInfo, SendMessageWirelessly, StateContext,
};

use my_hdlc::command::{self, DeviceCommand, DroneInfo, FSMState};
use my_hdlc::pc_command::{ManualDroneInput, ManualDroneTrimsEnums};
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
const WATCHDOG_TIMER_FOR_PANICKING: Duration = Duration::from_millis(1000);
const DRONE_STATE_TIMER: [Duration; 2] = [Duration::from_millis(20), Duration::from_millis(5)];
const SAVE_TO_LOG_TIMER: Duration = Duration::from_millis(100);

const SHOULD_CHECK_BATTERY_LEVEL: bool = true;
const MIN_BAT_LEVEL: u16 = 700;

const BATTERY_ALPHA: I16F16 = I16F16::lit("0.1");
const BATTERY_BETA: I16F16 = I16F16::lit("0.9");

// -------------------------------------------------------------------------

pub fn main_loop() -> ! {
    // let start = Instant::now();
    // loop {
    //     if Instant::now().duration_since(start) >= Duration::from_secs(6) {
    //         break;
    //     }
    // }

    // processor tick frequency
    set_tick_frequency(500);
    // -------------------------------------------------------------------------

    // buffer for receiving bytes from PC
    let mut receive_buffer = Box::new([0u8; UART_BUF_SIZE]);

    // -------------------------------------------------------------------------

    // current state in FSM
    let mut current_state: Box<dyn FSMControl> = Box::new(FSMSafe {});

    // -------------------------------------------------------------------------

    // Time data for telemetry data
    let current_time = Instant::now();
    // -------------------------------------------------------------------------

    // fields for the context
    let mut transceiver: Box<HdlcTransceiver> = Box::new(HdlcTransceiver::new());

    let mut manual_trim: ManualDroneTrimsEnums = ManualDroneTrimsEnums::default();
    let mut input_as_ypr: YawPitchRoll<I16F16, I16F16> = YawPitchRoll::<I16F16, I16F16>::new();

    let mut calibration_state: CalibrationState<I16F16, I16F16> =
        CalibrationState::<I16F16, I16F16>::new();

    let mut pressure_sensor_filter = PressureSensor::new();
    let mut position_kalman = KalmanFilter::new((
        calibration_state.accelerometer_offset,
        calibration_state.gyro_offset,
    ));
    let mut dmp_sampler = DmpReadings::new(
        calibration_state.ypr_offset,
        calibration_state.accelerometer_offset,
    );

    let mut pid_info: Box<PIDInfo> = Box::new(PIDInfo {
        selected_height: 0f32,
    });

    let mut ctx = StateContext {
        kalman_position: position_kalman,
        calibration_state: &mut calibration_state,
        pressure_sensor_filter: &mut pressure_sensor_filter,
        dmp_filter: dmp_sampler,

        pid_controller: PIDController::<I16F16, I16F16>::new(),

        trv: &mut transceiver,

        input_as_ypr: &mut input_as_ypr,

        flash_head: 0usize,
        flash_tail: 0usize,

        time_for_main_loop: 0i32,

        pid_info: &mut pid_info,

        is_wireless: false,
        curent_state: FSMState::SafeMode,
        dt: Duration::from_millis(0),
        wireless_log: SendMessageWirelessly::default(),
    };

    // -------------------------------------------------------------------------

    // used for determining whether we should panic
    let mut time_for_last_received_message: Instant = Instant::now();
    let mut last_send_message: Instant = Instant::now();
    let mut last_logged_message: Instant = Instant::now();

    let mut time_previous_loop = Instant::now();

    let mut bat_level = I16F16::from_num(read_battery());

    // -------------------------------------------------------------------------
    for i in 0.. {
        let _ = Blue.toggle();
        // -------------------------------------------------------------------------
        // Check battery level and switch to panic
        bat_level = BATTERY_ALPHA * I16F16::from_num(read_battery()) + BATTERY_BETA * bat_level;
        if SHOULD_CHECK_BATTERY_LEVEL && bat_level < MIN_BAT_LEVEL {
            current_state = current_state.step(command::FSMState::PanicMode, &mut ctx);
        }
        // -------------------------------------------------------------------------

        // Read Uart Buff
        let num_received = {
            if ctx.is_wireless {
                BLE_BUFFER.modify(|x| {
                    for i in 0..x.1 {
                        receive_buffer[i] = x.0[i];
                    }
                    let size_to_return: usize = x.1;
                    x.1 = 0;
                    size_to_return
                })
            } else {
                receive_bytes(&mut receive_buffer[0..ctx.trv.bytes_to_read()])
            }
        };

        if num_received != 0usize {
            ctx.trv.add_bytes(&receive_buffer[..num_received]);

            //try to deserialize the command
            let deserialized_command = ctx.trv.read_structure::<DeviceCommand>();

            // if there is a command
            if let Some(command) = deserialized_command {
                // Red.toggle();
                match command {
                    DeviceCommand::ChangeMode(new_mode) => {
                        current_state = current_state.step(new_mode, &mut ctx);
                        // send_ack(&mut ctx.trv);
                    }
                    DeviceCommand::ManualInput(manual_input) => {
                        *ctx.input_as_ypr =
                            YawPitchRoll::<I16F16, I16F16>::from_manual_input(&manual_input);
                    }
                    DeviceCommand::ManualDroneTrims(manual_trims) => {
                        let (ind, values) = match manual_trims {
                            ManualDroneTrimsEnums::Lift(values) => (3, values),

                            ManualDroneTrimsEnums::Yaw(values) => (0, values),

                            ManualDroneTrimsEnums::Pitch(values) => (1, values),

                            ManualDroneTrimsEnums::Roll(values) => (2, values),
                        };
                        ctx.pid_controller.k_p[ind] = ControllerValues::from_num(values.p_value);

                        ctx.pid_controller.k_i[ind] = ControllerValues::from_num(values.i_value);

                        ctx.pid_controller.k_d[ind] = ControllerValues::from_num(values.d_value);
                    }
                    _ => {}
                }
                time_for_last_received_message = Instant::now();
            }
        }

        // update filter readings
        ctx.kalman_position.append_new_reading();

        ctx.pressure_sensor_filter
            .update_readings(&mut ctx.kalman_position);

        // run the loop of the state
        current_state = current_state.run_state_loop(&mut ctx);

        let time_end = Instant::now();

        if time_end.duration_since(time_for_last_received_message) >= WATCHDOG_TIMER_FOR_PANICKING {
            ctx.is_wireless ^= true;
            current_state = current_state.step(command::FSMState::PanicMode, &mut ctx);
        }

        let dt = time_end.duration_since(current_time);

        //-----------------------------
        // for loggign
        ctx.curent_state = current_state.get_state();
        ctx.time_for_main_loop = time_end.duration_since(time_previous_loop).as_millis() as i32;
        ctx.dt = dt;
        //-----------------------------

        time_previous_loop = time_end;

        if time_end.duration_since(last_send_message) >= DRONE_STATE_TIMER[ctx.is_wireless as usize]
        {
            last_send_message = time_end;

            send_drone_data(&mut ctx);
        }

        if ctx.is_wireless && time_end.duration_since(last_logged_message) >= SAVE_TO_LOG_TIMER {
            last_logged_message = time_end;
            put_telemetry_data_on_flash(&mut ctx);
        }

        // -------------------------------------------------------------------------

        wait_for_next_tick();
    }
    unreachable!();
}

/*
* Sends data to the drone
*/
fn send_drone_data(ctx: &mut StateContext) {
    Green.on();

    let readers: [fn(&mut StateContext, bool, bool) -> ReturnType; 7] = [
        <TelemetryData as TelemetryRead>::read_general_data,
        <TelemetryData as TelemetryRead>::read_motor_data,
        <TelemetryData as TelemetryRead>::read_position_data,
        <TelemetryData as TelemetryRead>::read_pressure_data,
        <TelemetryData as TelemetryRead>::read_raw_data,
        <TelemetryData as TelemetryRead>::read_pid_info,
        <TelemetryData as TelemetryRead>::read_calibration_info,
    ];

    let mut msg = ([0u8; STUFFED_MESSAGE_SIZE], 0usize);

    let should_send_wireless = ctx.is_wireless ^ ctx.wireless_log.forced_message;

    if should_send_wireless {
        // we can send only one packet of size 20 bytes
        let mut msg_to_read = ctx.wireless_log.message_ind;
        let mut part_to_read = ctx.wireless_log.message_part;

        if part_to_read == 0 {
            msg = readers[msg_to_read](ctx, false, true);

            ctx.wireless_log.stored_msg = msg.0;
            ctx.wireless_log.message_len = msg.1;
        } else {
            msg.0 = ctx.wireless_log.stored_msg;
            msg.1 = ctx.wireless_log.message_len;
        }

        let end_of_message = (part_to_read + BLE_BUFFER_SIZE).min(msg.1);
        let message_len = end_of_message - part_to_read;

        unsafe {
            let mut to_send = [0u8; BLE_BUFFER_SIZE];
            for i in 0..message_len {
                to_send[i] = msg.0[i + part_to_read];
            }

            ble_send(to_send.as_ptr(), message_len as u16);
        }

        if (end_of_message == msg.1) {
            if ctx.wireless_log.message_ind == 0 {
                ctx.wireless_log.forced_message = false;
            }

            ctx.wireless_log.message_ind = (ctx.wireless_log.message_ind + 1) % 7;
            ctx.wireless_log.message_part = 0;
        } else {
            ctx.wireless_log.message_part = end_of_message;
        }
    } else {
        ctx.wireless_log.forced_message = false;
        for reader in readers {
            msg = reader(ctx, false, true);
            send_bytes(&msg.0[0..msg.1]);
        }
    }

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

fn put_telemetry_data_on_flash(ctx: &mut StateContext) {
    if ctx.flash_tail + (STUFFED_MESSAGE_SIZE * 7) > 0x01FFFF {
        return;
    }

    Yellow.on();

    let readers: [fn(&mut StateContext, bool, bool) -> ReturnType; 7] = [
        <TelemetryData as TelemetryRead>::read_general_data,
        <TelemetryData as TelemetryRead>::read_motor_data,
        <TelemetryData as TelemetryRead>::read_position_data,
        <TelemetryData as TelemetryRead>::read_pressure_data,
        <TelemetryData as TelemetryRead>::read_raw_data,
        <TelemetryData as TelemetryRead>::read_pid_info,
        <TelemetryData as TelemetryRead>::read_calibration_info,
    ];

    let mut msg = ([0u8; STUFFED_MESSAGE_SIZE], 0usize);

    for reader in readers {
        msg = reader(ctx, true, false);
        _ = flash_write_bytes(ctx.flash_tail as u32, &msg.0[0..msg.1]);
        ctx.flash_tail += STUFFED_MESSAGE_SIZE;
    }

    Yellow.off();
}
