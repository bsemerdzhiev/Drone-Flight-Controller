use tudelft_quadrupel::mpu::structs::{Accel, Gyro};

const LOOK_BACK_ELEMENTS: i32 = 100;

struct ButterWorth {
    output: (Accel, Gyro),
    prev_input: (Accel, Gyro),
}

impl ButterWorth {
    fn new(input: (Accel, Gyro)) -> Self {
        Self {
            output: input,
            prev_input: input,
        }
    }
    fn append_new_reading(&mut self, input: (Accel, Gyro)) {
        let cur_input_arr: [i16; 6] = [
            input.0.x, input.0.y, input.0.z, input.1.x, input.1.y, input.1.z,
        ];
        let prev_output_arr: [i16; 6] = [
            self.output.0.x,
            self.output.0.y,
            self.output.0.z,
            self.output.1.x,
            self.output.1.y,
            self.output.1.z,
        ];
        let prev_input_arr: [i16; 6] = [
            self.prev_input.0.x,
            self.prev_input.0.y,
            self.prev_input.0.z,
            self.prev_input.1.x,
            self.prev_input.1.y,
            self.prev_input.1.z,
        ];

        let mut cur_output: [i16; 6] = [0, 0, 0, 0, 0, 0];

        for i in 0..cur_output.len() {
            let y_prev: i32 = prev_output_arr[i] as i32;
            let x_prev: i32 = prev_input_arr[i] as i32;
            let x_cur: i32 = cur_input_arr[i] as i32;

            let y_cur: i32 = (((LOOK_BACK_ELEMENTS - 1i32) * y_prev) / LOOK_BACK_ELEMENTS)
                + (x_prev / 2i32)
                + (x_cur / 2i32);

            cur_output[i] = y_cur.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        }

        self.output = (
            Accel {
                x: cur_output[0],
                y: cur_output[1],
                z: cur_output[2],
            },
            Gyro {
                x: cur_output[3],
                y: cur_output[4],
                z: cur_output[5],
            },
        );

        self.prev_input = input;
    }
}
