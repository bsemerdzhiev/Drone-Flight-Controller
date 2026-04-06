use core::time::Duration;

use crate::filters::sensors_handler::ImuHandler;
use crate::states::state_structures::state_context::StateContext;
use crate::util::yaw_pitch_roll::YawPitchRoll;
use fixed::types::I16F16;
use fixed::types::I26F6;
use fixed::types::I32F0;
use fixed::types::I4F28;
use fixed::types::I8F24;
use my_hdlc::command::DeviceCommand;
use my_hdlc::command::FSMState;
use my_hdlc::telemetry_data::CalibrationInfo;
use my_hdlc::telemetry_data::PIDInfo;
use my_hdlc::telemetry_data::{
    GeneralData, MotorData, PositionData, PressureData, RawData, TelemetryData,
    TELEMETERY_DATA_SIZE,
};
use my_hdlc::STUFFED_MESSAGE_SIZE;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::block;
use tudelft_quadrupel::led::Led::Yellow;
use tudelft_quadrupel::motor::get_motors;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw, structs::*};

pub type ReturnType = (([u8; STUFFED_MESSAGE_SIZE], usize));

pub trait TelemetryRead {
    fn read_general_data(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType;

    fn read_motor_data(ctx: &mut StateContext, logged_in_flash: bool, encoded: bool) -> ReturnType;

    fn read_position_data(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType;

    fn read_raw_data(ctx: &mut StateContext, logged_in_flash: bool, encoded: bool) -> ReturnType;

    fn read_pressure_data(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType;

    fn read_pid_info(ctx: &mut StateContext, logged_in_flash: bool, encoded: bool) -> ReturnType;

    fn read_calibration_info(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType;
}

impl TelemetryRead for TelemetryData {
    fn read_general_data(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType {
        let bat = read_battery();

        let data_to_send = DeviceCommand::Telemetry(TelemetryData::GeneralData(GeneralData {
            logged_in_flash: logged_in_flash,
            dt: ctx.dt.as_millis() as u32,
            time_for_main_loop: ctx.time_for_main_loop,

            com_mode: ctx.is_wireless,

            bat: bat,
            cur_state: ctx.curent_state,
            is_wireless: ctx.is_wireless,
        }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }

    fn read_motor_data(ctx: &mut StateContext, logged_in_flash: bool, encoded: bool) -> ReturnType {
        let data_to_send = DeviceCommand::Telemetry(TelemetryData::MotorData(MotorData {
            logged_in_flash: logged_in_flash,
            motors: get_motors(),
        }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }

    fn read_position_data(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType {
        let quaternion = block!(read_dmp_bytes());

        let kalman_pos = ctx.kalman_position.get_reading::<I16F16, I16F16>();

        let mut ypr: YawPitchRoll<I16F16, I16F16> = if quaternion.is_ok() {
            YawPitchRoll::<I16F16, I16F16>::from(quaternion.unwrap())
        } else {
            YawPitchRoll::new()
        };
        ypr.pitch -= I16F16::from_num(ctx.calibration_state.ypr_offset.pitch);
        ypr.roll -= I16F16::from_num(ctx.calibration_state.ypr_offset.roll);
        ypr.yaw -= I16F16::from_num(ctx.calibration_state.ypr_offset.yaw);

        // ypr.yaw -=
        // (ctx.calibration_state.gyro_offset.z as f32) * micromath::F32Ext::acos(-1.0) / 180.0;

        let data_to_send = DeviceCommand::Telemetry(TelemetryData::PositionData(PositionData {
            logged_in_flash: logged_in_flash,

            yaw: ypr.yaw.to_num::<f32>(),
            pitch: ypr.pitch.to_num::<f32>(),
            roll: ypr.roll.to_num::<f32>(),

            yaw_kalman: kalman_pos.yaw.to_num::<f32>(),
            pitch_kalman: kalman_pos.pitch.to_num::<f32>(),
            roll_kalman: kalman_pos.roll.to_num::<f32>(),
        }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }

    fn read_pressure_data(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType {
        let pres = ctx
            .pressure_sensor_filter
            .pressure_to_meters(I32F0::from_num(read_pressure() as i32));
        let data_to_send = DeviceCommand::Telemetry(TelemetryData::PressureData(PressureData {
            logged_in_flash: logged_in_flash,

            pres: pres.to_num::<f32>(),
            pressure_filtered: ctx.pressure_sensor_filter.get_reading().to_num::<f32>(),
        }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }

    fn read_raw_data(ctx: &mut StateContext, logged_in_flash: bool, encoded: bool) -> ReturnType {
        let (accel_raw, gyro_raw) = read_raw().unwrap();

        let data_to_send = DeviceCommand::Telemetry(TelemetryData::RawData(RawData {
            logged_in_flash: logged_in_flash,

            accel_x: accel_raw.x,
            accel_y: accel_raw.y,
            accel_z: accel_raw.z,

            gyro_x: gyro_raw.x,
            gyro_y: gyro_raw.y,
            gyro_z: gyro_raw.z,
        }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }

    fn read_pid_info(ctx: &mut StateContext, logged_in_flash: bool, encoded: bool) -> ReturnType {
        let data_to_send = DeviceCommand::Telemetry(TelemetryData::PIDInfo(PIDInfo {
            logged_in_flash: logged_in_flash,

            selected_height: ctx.pid_info.selected_height,
        }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }

    fn read_calibration_info(
        ctx: &mut StateContext,
        logged_in_flash: bool,
        encoded: bool,
    ) -> ReturnType {
        let data_to_send =
            DeviceCommand::Telemetry(TelemetryData::CalibrationInfo(CalibrationInfo {
                logged_in_flash: logged_in_flash,

                averaged_accel: ctx.calibration_state.accelerometer_offset.to_array(),
                averaged_gyro: ctx.calibration_state.gyro_offset.to_array(),
                averaged_ypr: ctx.calibration_state.ypr_offset.to_array(),
            }));

        if encoded {
            return ctx.trv.write_structure(&data_to_send);
        } else {
            let mut buf = [0u8; STUFFED_MESSAGE_SIZE];
            let len = postcard::to_slice(&data_to_send, &mut buf).unwrap().len();
            return (buf, len);
        }
    }
}
