use tudelft_quadrupel::mpu::structs::{Accel, Gyro};

pub struct CalibrationState {
    accel: Axis<i16>,
    gyro: Axis<i16>,
    accel_sum: Axis<i32>,
    gyro_sum: Axis<i32>,
    samples: u32,
    calib_done: bool,
}

#[derive(Copy, Clone, Debug, PartialEq, Default)]
pub struct Axis<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl From<Accel> for Axis<i16> {
    fn from(a: Accel) -> Self {
        Axis::<i16> {
            x: a.x,
            y: a.y,
            z: a.z,
        }
    }
}

impl From<Gyro> for Axis<i16> {
    fn from(g: Gyro) -> Self {
        Axis::<i16> {
            x: g.x,
            y: g.y,
            z: g.z,
        }
    }
}

impl CalibrationState {
    pub fn new() -> Self {
        return CalibrationState {
            accel: Axis::<i16> { x: 0, y: 0, z: 0 },
            gyro: Axis::<i16> { x: 0, y: 0, z: 0 },
            accel_sum: Axis::<i32>::default(),
            gyro_sum: Axis::<i32>::default(),
            samples: 0,
            calib_done: false,
        };
    }
    pub fn start_calibration(&mut self) {
        self.accel = Axis::<i16> { x: 0, y: 0, z: 0 };
        self.gyro = Axis::<i16> { x: 0, y: 0, z: 0 };
        self.accel_sum = Axis::<i32>::default();
        self.gyro_sum = Axis::<i32>::default();
        self.samples = 0;
        self.calib_done = false;
    }

    pub fn accumulate_calibration(&mut self, accel: Axis<i16>, gyro: Axis<i16>) {
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
        self.accel = Axis::<i16> {
            x: (self.accel_sum.x / n) as i16,
            y: (self.accel_sum.y / n) as i16,
            z: (self.accel_sum.z / n) as i16,
        };

        self.gyro = Axis::<i16> {
            x: (self.gyro_sum.x / n) as i16,
            y: (self.gyro_sum.y / n) as i16,
            z: (self.gyro_sum.z / n) as i16,
        };
        self.calib_done = true;
    }

    pub fn apply_calibration(
        &mut self,
        accel: Axis<i16>,
        gyro: Axis<i16>,
    ) -> (Axis<i16>, Axis<i16>) {
        if !self.calib_done {
            // Return raw data if drone not calibrated yet (failsafe)
            return (accel, gyro);
        }

        // Remove the offsets here for each axis
        (
            Axis::<i16> {
                //True value = measured - offset (m=tv + off)
                x: accel.x.wrapping_sub(self.accel.x), //wrapping_sub because subrtacting i16 can overflow
                y: accel.y.wrapping_sub(self.accel.y),
                z: accel.z.wrapping_sub(self.accel.z),
            },
            Axis::<i16> {
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
                Axis::<i16> {
                    x: 100,
                    y: -50,
                    z: 16380,
                },
                Axis::<i16> { x: 12, y: -8, z: 4 },
            ), //(Accel, Gyro)
            (
                Axis::<i16> {
                    x: 101,
                    y: -49,
                    z: 16382,
                },
                Axis::<i16> { x: 11, y: -7, z: 3 },
            ),
            (
                Axis::<i16> {
                    x: 99,
                    y: -51,
                    z: 16381,
                },
                Axis::<i16> { x: 13, y: -9, z: 5 },
            ),
            (
                Axis::<i16> {
                    x: 100,
                    y: -50,
                    z: 16379,
                },
                Axis::<i16> { x: 12, y: -8, z: 4 },
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
            Axis::<i16> {
                x: 100,
                y: -50,
                z: 16380,
            },
            Axis::<i16> { x: 12, y: -8, z: 4 },
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

        let accel = Axis::<i16> {
            x: 100,
            y: -50,
            z: 16380,
        };
        let gyro = Axis::<i16> { x: 12, y: -8, z: 4 };

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
