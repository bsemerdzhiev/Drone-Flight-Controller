use tudelft_quadrupel::mpu::{
    read_dmp_bytes, read_raw,
    structs::{Accel, Gyro, Quaternion},
};

pub struct SensorState {
    quaternion: Quaternion,
    accel: Accel,
    gyro: Gyro,
}

impl SensorState {
    pub fn new() -> Self {
        let (accel, gyro) = read_raw().unwrap();
        return SensorState {
            quaternion: block!(read_dmp_bytes()).unwrap(),
            accel,
            gyro,
        };
    }

    pub fn update_quaternion(&mut self) {
        self.quaternion = block!(read_dmp_bytes()).unwrap();
    }

    pub fn update_raw_data(&mut self) {
        let (accel, gyro) = read_raw().unwrap();
        self.accel = accel;
        self.gyro = gyro;
    }

    pub fn get_quaternion(&self) -> Quaternion {
        return self.quaternion;
    }
    pub fn get_raw_data(&self) -> (Accel, Gyro) {
        return (self.accel, self.gyro);
    }
}
