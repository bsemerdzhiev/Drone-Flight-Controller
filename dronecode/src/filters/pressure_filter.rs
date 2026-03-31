use core::time::Duration;

use micromath::F32Ext;
use nalgebra::{Matrix1x2, Matrix2, Matrix2x1, Vector2};
use tudelft_quadrupel::{barometer::read_pressure, mpu::read_raw, time::Instant};

use crate::{
    filters::{
        kalman_filter::{self, KalmanFilter},
        sensors_handler::ImuHandler,
    },
    states::state_structures::calibration_state::LSB_FOR_ACCEL,
};
type Scalar = f32;

const EPS: f32 = 1e-6;
const THRESHOLD_BARO: f32 = 1.5f32;

// https://www.alldatasheet.com/datasheet-pdf/download/1132807/TDK/MPU-6050.html
// Page 12
// Accelerometer sample rate is set to 1kHz
const ACCEL_SAMPLE_RATE: Duration = Duration::from_millis(1);

// TUDelft's library initializes it with the corresponding oversampling ratio
const BARO_SAMPLE_DURATION: Duration = Duration::from_millis(10);

pub struct PressureSensor {
    // ------------------------------------
    current_state: Matrix2x1<f32>,
    observation_matrix: Matrix1x2<f32>,
    uncertainty_matrix: Matrix2<f32>,

    // ------------------------------------
    barometer_variance: f32,
    accelerometer_variance: f32,

    // ------------------------------------
    last_barometer: f32,
    last_accelerometer: f32,

    last_reading_accel: Instant,
    last_reading_baro: Instant,

    pub baseline_pressure: i32,
}

impl PressureSensor {
    pub fn new() -> Self {
        Self {
            current_state: Vector2::zeros(),
            observation_matrix: Matrix1x2::new(1.0, 0.0),
            uncertainty_matrix: Matrix2::identity(),

            accelerometer_variance: 0.6f32,
            barometer_variance: 5e-2,

            last_barometer: 0.0,
            last_accelerometer: 1.0,

            last_reading_accel: Instant::now(),
            last_reading_baro: Instant::now(),

            baseline_pressure: 101325,
        }
    }

    pub fn reset(&mut self) {
        self.baseline_pressure = read_pressure() as i32;
        self.current_state = Vector2::zeros();
        self.uncertainty_matrix = Matrix2::identity();
    }

    pub fn pressure_to_meters(&mut self, pressure_reading: i32) -> f32 {
        // NOTE: more physics accurate formula
        // return 44330.0
        //     * (1.0
        //         - (micromath::F32Ext::powf(
        //             pressure_reading as f32 / self.baseline_pressure as f32,
        //             (1.0 / 5.255),
        //         )));
        return (-pressure_reading + self.baseline_pressure) as f32 / 12f32;
    }

    pub fn prediction(&mut self, filtered_position: &mut KalmanFilter) {
        let cur_time = Instant::now();
        if (cur_time.duration_since(self.last_reading_accel) < ACCEL_SAMPLE_RATE) {
            return;
        }

        let mut raw_accel = read_raw().unwrap().0;

        let mut raw_accel_x =
            (raw_accel.x - filtered_position.calibration_offset.0.x) as f32 / LSB_FOR_ACCEL as f32;
        let mut raw_accel_y =
            (raw_accel.y - filtered_position.calibration_offset.0.y) as f32 / LSB_FOR_ACCEL as f32;
        let mut raw_accel_z =
            (raw_accel.z - filtered_position.calibration_offset.0.z) as f32 / LSB_FOR_ACCEL as f32;

        let mut accel_input = (-raw_accel_x * micromath::F32Ext::sin(filtered_position.pitch))
            + (raw_accel_y
                * micromath::F32Ext::sin(filtered_position.roll)
                * micromath::F32Ext::cos(filtered_position.pitch))
            + (raw_accel_z
                * micromath::F32Ext::cos(filtered_position.roll)
                * micromath::F32Ext::cos(filtered_position.pitch));

        accel_input -= 1.0;

        accel_input *= 9.81;
        // ----------------------------------------------------------------------
        /*
         *  Math is:
         *   x_(k+1) = F*x_k + B*u_k
         *   P_(k+1) = F*P*F^T + Q
         */
        let dt: f32 = cur_time
            .duration_since(self.last_reading_accel)
            .as_secs_f32()
            .clamp(0.001, 0.02);

        let control_input_matrix: Matrix2x1<f32> = Matrix2x1::new(dt * dt * 0.5, dt);

        let transition_matrix = Matrix2::new(1.0, dt, 0.0, 1.0);

        let process_uncertainty = Matrix2::new(
            dt * dt * dt * dt * 0.25,
            dt * dt * dt * 0.5,
            dt * dt * dt * 0.5,
            dt * dt,
        ) * self.accelerometer_variance;

        self.current_state =
            (transition_matrix * self.current_state) + (control_input_matrix * accel_input);

        self.uncertainty_matrix = ((transition_matrix * self.uncertainty_matrix)
            * transition_matrix.transpose())
            + process_uncertainty;

        self.last_accelerometer = accel_input;
        self.last_reading_accel = cur_time;
    }

    pub fn correction(&mut self) {
        let cur_time = Instant::now();
        if (cur_time.duration_since(self.last_reading_baro) < BARO_SAMPLE_DURATION) {
            return;
        }

        let baro_reading = self.pressure_to_meters(read_pressure() as i32);

        let mut kalman_gain: Matrix2x1<f32> =
            self.uncertainty_matrix * self.observation_matrix.transpose();

        let inovation_variance = (((self.observation_matrix * self.uncertainty_matrix)
            * self.observation_matrix.transpose())
        .x + self.barometer_variance);

        kalman_gain = kalman_gain / inovation_variance;

        let inovation = (baro_reading - (self.observation_matrix * self.current_state).x);

        // if inovation.abs() > 10.0 * inovation_variance.sqrt() {
        //     return;
        // }

        self.current_state = self.current_state + (kalman_gain * inovation);

        let i = Matrix2::identity();
        self.uncertainty_matrix = (i - kalman_gain * self.observation_matrix)
            * self.uncertainty_matrix
            * (i - kalman_gain * self.observation_matrix).transpose()
            + kalman_gain * self.barometer_variance * kalman_gain.transpose();

        self.last_barometer = baro_reading;
        self.last_reading_baro = cur_time;
    }

    pub fn update_readings(&mut self, filtered_position: &mut KalmanFilter) {
        self.prediction(filtered_position);
        self.correction();
    }

    pub fn get_reading(&self) -> f32 {
        return self.current_state.x;
    }
}
