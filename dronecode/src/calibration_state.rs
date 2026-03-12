pub struct CalibrationState {
    accel: Axis,
    gyro: Axis,
    accel_sum: AxisI32,
    gyro_sum: AxisI32,
    samples: u32,
    calib_done: bool,
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Axis {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Copy, Clone, Debug, Default)]
struct AxisI32 {
    //Only used for the sums in accumulate_calibration
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl CalibrationState {
    pub fn new() -> Self {
        return CalibrationState {
            accel: Axis { x: 0, y: 0, z: 0 },
            gyro: Axis { x: 0, y: 0, z: 0 },
            accel_sum: AxisI32::default(),
            gyro_sum: AxisI32::default(),
            samples: 0,
            calib_done: false,
        };
    }
    pub fn start_calibration(&mut self) {
        self.accel = Axis { x: 0, y: 0, z: 0 };
        self.gyro = Axis { x: 0, y: 0, z: 0 };
        self.accel_sum = AxisI32::default();
        self.gyro_sum = AxisI32::default();
        self.samples = 0;
        self.calib_done = false;
    }

    pub fn accumulate_calibration(&mut self, accel: Axis, gyro: Axis) {
        //Collects the sensor data
        self.accel_sum.x += accel.x as i32;
        self.accel_sum.y += accel.y as i32;
        self.accel_sum.z += accel.z as i32;

        self.gyro_sum.x += gyro.x as i32;
        self.gyro_sum.y += gyro.y as i32;
        self.gyro_sum.z += gyro.z as i32;
        self.samples += 1;
    }

    pub fn finish_calibration(&mut self) {
        if self.samples == 0 {
            return;
        }

        let n = self.samples as i32;
        self.accel = Axis {
            x: (self.accel_sum.x / n) as i16,
            y: (self.accel_sum.y / n) as i16,
            z: (self.accel_sum.z / n) as i16,
        };

        self.gyro = Axis {
            x: (self.gyro_sum.x / n) as i16,
            y: (self.gyro_sum.y / n) as i16,
            z: (self.gyro_sum.z / n) as i16,
        };
        self.calib_done = true;
    }

    pub fn apply_calibration(&mut self, accel: Axis, gyro: Axis) -> (Axis, Axis) {
        if !self.calib_done {
            // Return raw data if drone not calibrated yet (failsafe)
            return (accel, gyro);
        }

        // Remove the offsets here for each axis
        (
            Axis {
                //True value = measured - offset (m=tv + off)
                x: accel.x.wrapping_sub(self.accel.x), //wrapping_sub because subrtacting i16 can overflow
                y: accel.y.wrapping_sub(self.accel.y),
                z: accel.z.wrapping_sub(self.accel.z),
            },
            Axis {
                x: gyro.x.wrapping_sub(self.gyro.x), //wrapping_sub because subrtacting i16 can overflow
                y: gyro.y.wrapping_sub(self.gyro.y),
                z: gyro.z.wrapping_sub(self.gyro.z),
            },
        )
    }
}

#[cfg(test)]
mod tests {
    use super::{Axis, CalibrationState};

    #[test]
    fn calibration_removes_dc_offset() {
        let mut calib = CalibrationState::new();

        // Simulate raw values with offset
        let fake_samples = [
            (
                Axis {
                    x: 100,
                    y: -50,
                    z: 16380,
                },
                Axis { x: 12, y: -8, z: 4 },
            ), //(Accel, Gyro)
            (
                Axis {
                    x: 101,
                    y: -49,
                    z: 16382,
                },
                Axis { x: 11, y: -7, z: 3 },
            ),
            (
                Axis {
                    x: 99,
                    y: -51,
                    z: 16381,
                },
                Axis { x: 13, y: -9, z: 5 },
            ),
            (
                Axis {
                    x: 100,
                    y: -50,
                    z: 16379,
                },
                Axis { x: 12, y: -8, z: 4 },
            ),
        ];

        // Accumulate samples
        for (a, g) in fake_samples {
            calib.accumulate_calibration(a, g);
        }

        // Finish calibration (compute averages)
        calib.finish_calibration();

        // Apply calibration to a sample
        let (accel_corrected, gyro_corrected) = calib.apply_calibration(
            Axis {
                x: 100,
                y: -50,
                z: 16380,
            },
            Axis { x: 12, y: -8, z: 4 },
        );

        // Since drone was stationary, corrected values should be ~0
        assert!(accel_corrected.x.abs() <= 1);
        assert!(accel_corrected.y.abs() <= 1);
        assert!(accel_corrected.z.abs() <= 2);

        assert!(gyro_corrected.x.abs() <= 1);
        assert!(gyro_corrected.y.abs() <= 1);
        assert!(gyro_corrected.z.abs() <= 1);
    }

    #[test]
    fn apply_before_calibration_returns_raw_values() {
        let mut calib = CalibrationState::new();

        let accel = Axis {
            x: 100,
            y: -50,
            z: 16380,
        };
        let gyro = Axis { x: 12, y: -8, z: 4 };

        // Accumulate but DO NOT finish calibration
        calib.accumulate_calibration(accel, gyro);

        let (accel_out, gyro_out) = calib.apply_calibration(accel, gyro);

        // Raw values should be returned
        assert_eq!(accel_out.x, accel.x);
        assert_eq!(accel_out.y, accel.y);
        assert_eq!(accel_out.z, accel.z);

        assert_eq!(gyro_out.x, gyro.x);
        assert_eq!(gyro_out.y, gyro.y);
        assert_eq!(gyro_out.z, gyro.z);
    }
}
