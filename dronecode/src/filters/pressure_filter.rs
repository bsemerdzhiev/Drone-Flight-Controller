use core::time::Duration;

use cordic::{cos, sin};
use fixed::types::{I10F22, I16F16, I26F6, I2F30, I32F0, I4F28, I64F0, I8F24};
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

type MeasurementType = I8F24;

pub struct PressureSensor {
    // ------------------------------------
    current_state: Matrix2x1<MeasurementType>,
    observation_matrix: Matrix1x2<MeasurementType>,
    uncertainty_matrix: Matrix2<MeasurementType>,

    // ------------------------------------
    barometer_variance: MeasurementType,
    accelerometer_variance: MeasurementType,

    // ------------------------------------
    // last_barometer: SensorFixedType,
    // last_accelerometer: SensorFixedType,
    last_reading_accel: Instant,
    last_reading_baro: Instant,

    pub baseline_pressure: I32F0,
}

impl PressureSensor {
    pub fn new() -> Self {
        Self {
            current_state: Matrix2x1::new(
                MeasurementType::from_num(0),
                MeasurementType::from_num(0),
            ),
            observation_matrix: Matrix1x2::new(
                MeasurementType::from_num(1),
                MeasurementType::from_num(0),
            ),
            uncertainty_matrix: Matrix2::<MeasurementType>::new(
                MeasurementType::from_num(1),
                MeasurementType::from_num(0),
                MeasurementType::from_num(0),
                MeasurementType::from_num(1),
            ),

            accelerometer_variance: MeasurementType::from_num(0.6f32),
            barometer_variance: MeasurementType::from_num(5e-2),

            // last_barometer: ChosenFixedPointType::from_num(0.0),
            // last_accelerometer: ChosenFixedPointType::from_num(1.0),
            last_reading_accel: Instant::now(),
            last_reading_baro: Instant::now(),

            baseline_pressure: I32F0::from_num(101325),
        }
    }

    pub fn reset(&mut self) {
        self.baseline_pressure = I32F0::from_num(read_pressure());
        self.current_state =
            Matrix2x1::new(MeasurementType::from_num(0), MeasurementType::from_num(0));
        self.uncertainty_matrix = Matrix2::new(
            MeasurementType::from_num(1),
            MeasurementType::from_num(0),
            MeasurementType::from_num(0),
            MeasurementType::from_num(1),
        );
    }

    pub fn pressure_to_meters(&mut self, pressure_reading: I32F0) -> MeasurementType {
        // NOTE: more physics accurate formula
        // return 44330.0
        //     * (1.0
        //         - (micromath::F32Ext::powf(
        //             pressure_reading as f32 / self.baseline_pressure as f32,
        //             (1.0 / 5.255),
        //         )));
        return MeasurementType::from_num(-pressure_reading + self.baseline_pressure)
            / MeasurementType::from_num(12);
    }

    pub fn prediction(&mut self, filtered_position: &mut KalmanFilter) {
        let cur_time = Instant::now();
        if (cur_time.duration_since(self.last_reading_accel) < ACCEL_SAMPLE_RATE) {
            return;
        }

        let mut raw_accel = read_raw().unwrap().0;

        let mut raw_accel_x = MeasurementType::from_num(
            (I16F16::from_num(raw_accel.x) - filtered_position.calibration_offset.0.x)
                / I16F16::from_num(LSB_FOR_ACCEL),
        );

        let mut raw_accel_y = MeasurementType::from_num(
            (I16F16::from_num(raw_accel.y) - filtered_position.calibration_offset.0.y)
                / I16F16::from_num(LSB_FOR_ACCEL),
        );

        let mut raw_accel_z = MeasurementType::from_num(
            (I16F16::from_num(raw_accel.z) - filtered_position.calibration_offset.0.z)
                / I16F16::from_num(LSB_FOR_ACCEL),
        );

        // let mut accel_input: MeasurementType = raw_accel_z;

        //TODO: Bring back the world view below
        let mut accel_input: MeasurementType = (-raw_accel_x
            * MeasurementType::from_num(sin(filtered_position.pitch)))
            + (raw_accel_y
                * MeasurementType::from_num(sin(filtered_position.roll))
                * MeasurementType::from_num(cos(filtered_position.pitch)))
            + (raw_accel_z
                * MeasurementType::from_num(cos(filtered_position.roll))
                * MeasurementType::from_num(cos(filtered_position.pitch)));

        accel_input -= MeasurementType::from_num(1);

        accel_input *= MeasurementType::from_num(9.81);

        let dt: MeasurementType = MeasurementType::from_num(
            (I16F16::from_num(cur_time.duration_since(self.last_reading_accel).as_micros() as u32)
                / I16F16::from_num(1000))
                / I16F16::from_num(1000),
        );

        let control_input_matrix: Matrix2x1<MeasurementType> =
            Matrix2x1::new(dt * dt * MeasurementType::from_num(0.5), dt);

        let transition_matrix = Matrix2::<MeasurementType>::new(
            MeasurementType::from_num(1),
            dt,
            MeasurementType::from_num(0),
            MeasurementType::from_num(1),
        );

        let process_uncertainty = Matrix2::new(
            dt * dt * dt * dt * MeasurementType::from_num(0.25),
            dt * dt * dt * MeasurementType::from_num(0.5),
            dt * dt * dt * MeasurementType::from_num(0.5),
            dt * dt,
        )
        .map(|x| x * self.accelerometer_variance);

        // self.current_state = (transition_matrix * self.current_state);
        // self.current_state =
        //     (transition_matrix * self.current_state) + (control_input_matrix * accel_input);
        let new_state_0 = transition_matrix[(0, 0)] * self.current_state[(0, 0)]
            + transition_matrix[(0, 1)] * self.current_state[(1, 0)]
            + control_input_matrix[(0, 0)] * accel_input;

        let new_state_1 = transition_matrix[(1, 0)] * self.current_state[(0, 0)]
            + transition_matrix[(1, 1)] * self.current_state[(1, 0)]
            + control_input_matrix[(1, 0)] * accel_input;

        self.current_state = Matrix2x1::new(new_state_0, new_state_1);

        let tp = Matrix2::new(
            transition_matrix[(0, 0)] * self.uncertainty_matrix[(0, 0)]
                + transition_matrix[(0, 1)] * self.uncertainty_matrix[(1, 0)],
            transition_matrix[(0, 0)] * self.uncertainty_matrix[(0, 1)]
                + transition_matrix[(0, 1)] * self.uncertainty_matrix[(1, 1)],
            transition_matrix[(1, 0)] * self.uncertainty_matrix[(0, 0)]
                + transition_matrix[(1, 1)] * self.uncertainty_matrix[(1, 0)],
            transition_matrix[(1, 0)] * self.uncertainty_matrix[(0, 1)]
                + transition_matrix[(1, 1)] * self.uncertainty_matrix[(1, 1)],
        );

        let tpt = Matrix2::new(
            tp[(0, 0)] * transition_matrix[(0, 0)] + tp[(0, 1)] * transition_matrix[(0, 1)],
            tp[(0, 0)] * transition_matrix[(1, 0)] + tp[(0, 1)] * transition_matrix[(1, 1)],
            tp[(1, 0)] * transition_matrix[(0, 0)] + tp[(1, 1)] * transition_matrix[(0, 1)],
            tp[(1, 0)] * transition_matrix[(1, 0)] + tp[(1, 1)] * transition_matrix[(1, 1)],
        );

        self.uncertainty_matrix = Matrix2::new(
            tpt[(0, 0)] + process_uncertainty[(0, 0)],
            tpt[(0, 1)] + process_uncertainty[(0, 1)],
            tpt[(1, 0)] + process_uncertainty[(1, 0)],
            tpt[(1, 1)] + process_uncertainty[(1, 1)],
        );

        self.last_reading_accel = cur_time;
    }

    pub fn correction(&mut self) {
        let cur_time = Instant::now();
        if (cur_time.duration_since(self.last_reading_baro) < BARO_SAMPLE_DURATION) {
            return;
        }

        let baro_reading: MeasurementType =
            self.pressure_to_meters(I32F0::from_num(read_pressure() as i32));

        let mut kalman_gain: Matrix2x1<MeasurementType> = Matrix2x1::new(
            self.uncertainty_matrix[(0, 0)] * self.observation_matrix[(0, 0)]
                + self.uncertainty_matrix[(0, 1)] * self.observation_matrix[(0, 1)],
            self.uncertainty_matrix[(1, 0)] * self.observation_matrix[(0, 0)]
                + self.uncertainty_matrix[(1, 1)] * self.observation_matrix[(0, 1)],
        );

        let inovation_variance = self.uncertainty_matrix[(0, 0)] + self.barometer_variance;

        kalman_gain = kalman_gain / inovation_variance;

        // let inovation = (baro_reading - (self.observation_matrix * self.current_state)[(0, 0)]);
        let inovation = baro_reading - self.current_state[(0, 0)];

        self.current_state = self.current_state + (kalman_gain * inovation);

        // let i: Matrix2<MeasurementType> = Matrix2::<MeasurementType>::new(
        //     MeasurementType::from_num(1),
        //     MeasurementType::from_num(0),
        //     MeasurementType::from_num(0),
        //     MeasurementType::from_num(1),
        // );

        let k0 = kalman_gain[(0, 0)];
        let k1 = kalman_gain[(1, 0)];

        let i_minus_kh = Matrix2::new(
            MeasurementType::from_num(1) - k0,
            MeasurementType::from_num(0),
            -k1,
            MeasurementType::from_num(1),
        );

        // i_minus_kh * P * i_minus_kh^T
        let ikp = Matrix2::new(
            i_minus_kh[(0, 0)] * self.uncertainty_matrix[(0, 0)]
                + i_minus_kh[(0, 1)] * self.uncertainty_matrix[(1, 0)],
            i_minus_kh[(0, 0)] * self.uncertainty_matrix[(0, 1)]
                + i_minus_kh[(0, 1)] * self.uncertainty_matrix[(1, 1)],
            i_minus_kh[(1, 0)] * self.uncertainty_matrix[(0, 0)]
                + i_minus_kh[(1, 1)] * self.uncertainty_matrix[(1, 0)],
            i_minus_kh[(1, 0)] * self.uncertainty_matrix[(0, 1)]
                + i_minus_kh[(1, 1)] * self.uncertainty_matrix[(1, 1)],
        );

        let ikpiT = Matrix2::new(
            ikp[(0, 0)] * i_minus_kh[(0, 0)] + ikp[(0, 1)] * i_minus_kh[(0, 1)],
            ikp[(0, 0)] * i_minus_kh[(1, 0)] + ikp[(0, 1)] * i_minus_kh[(1, 1)],
            ikp[(1, 0)] * i_minus_kh[(0, 0)] + ikp[(1, 1)] * i_minus_kh[(0, 1)],
            ikp[(1, 0)] * i_minus_kh[(1, 0)] + ikp[(1, 1)] * i_minus_kh[(1, 1)],
        );

        // K * R * K^T where R is scalar barometer_variance
        let r = self.barometer_variance;
        let kkT = Matrix2::new(k0 * k0 * r, k0 * k1 * r, k1 * k0 * r, k1 * k1 * r);

        self.uncertainty_matrix = Matrix2::new(
            ikpiT[(0, 0)] + kkT[(0, 0)],
            ikpiT[(0, 1)] + kkT[(0, 1)],
            ikpiT[(1, 0)] + kkT[(1, 0)],
            ikpiT[(1, 1)] + kkT[(1, 1)],
        );

        self.last_reading_baro = cur_time;
    }

    pub fn update_readings(&mut self, filtered_position: &mut KalmanFilter) {
        self.prediction(filtered_position);
        self.correction();
    }

    pub fn get_reading(&self) -> I16F16 {
        return I16F16::from_num(self.current_state[(0, 0)]);
    }
}
