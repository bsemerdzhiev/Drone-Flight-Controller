use tudelft_quadrupel::block;
use tudelft_quadrupel::mpu::{
    read_dmp_bytes, read_raw,
    structs::{Accel, Gyro},
};
use tudelft_quadrupel::nrf51_pac::qdec::sample;
pub struct CalibrationState {
    accel_x: i32,
    accel_y: i32,
    accel_z: i32,
    gyro_x: i32,
    gyro_y: i32,
    gyro_z: i32,
    samples: u32,
    calib_done: bool,
}

impl CalibrationState {
    pub fn new() -> Self {
        return CalibrationState {
            accel_x: (0),
            accel_y: (0),
            accel_z: (0),
            gyro_x: (0),
            gyro_y: (0),
            gyro_z: (0),
            samples: (0),
            calib_done: false,
        };
    }
    pub fn start_calibration(&mut self) {
        self.accel_x = 0;
        self.accel_y = 0;
        self.accel_z = 0;
        self.gyro_x = 0;
        self.gyro_y = 0;
        self.gyro_z = 0;
        self.samples = 0;
        self.calib_done = false;
    }

    pub fn accumulate_calibration(&mut self, accel: Accel, gyro: Gyro) {
        //Collects the sensor data
        self.accel_x += accel.x as i32;
        self.accel_y += accel.y as i32;
        self.accel_z += accel.z as i32;
        self.gyro_x += gyro.x as i32;
        self.gyro_y += gyro.y as i32;
        self.gyro_z += gyro.z as i32;
        self.samples += 1;
    }

    pub fn finish_calibration(&mut self) {
        if self.samples == 0 {
            return;
        }
        self.accel_x /= self.samples as i32;
        self.accel_y /= self.samples as i32;
        self.accel_z /= self.samples as i32;
        self.gyro_x /= self.samples as i32;
        self.gyro_y /= self.samples as i32;
        self.gyro_z /= self.samples as i32;
        self.calib_done = true;
    }

    pub fn apply_calibration(&mut self, accel: Accel, gyro: Gyro) -> (Accel, Gyro) {
        if !self.calib_done {
            // Return raw data if drone not calibrated yet (failsafe)
            return (accel, gyro);
        }

        // Remove the offsets here for each axis
        let ret_accel = Accel {
            //True value = measured - offset (m=tv + off)
            x: accel.x - self.accel_x as i16,
            y: accel.y - self.accel_y as i16,
            z: accel.z - self.accel_z as i16,
        };
        let ret_gyro = Gyro {
            x: gyro.x - self.gyro_x as i16,
            y: gyro.y - self.gyro_y as i16,
            z: gyro.z - self.gyro_z as i16,
        };
        return (ret_accel, ret_gyro);
    }
}
