use micromath::F32Ext;
use my_hdlc::{
    command::{DebugRpms, DeviceCommand},
    pc_command::ManualInput,
};
use tudelft_quadrupel::{motor, uart::send_bytes};

const THRUST_COEFFICIENT: f32 = 14e-8;
const DRAG_COEFFICIENT: f32 = 2e-6;

const MIN_PWM: u16 = 200;

const MAX_RPMS: f32 = (980 * 10) as f32;

const THRESHOLD_LIFT: i32 = 1;

fn map_rpm_square_to_pwm(
    lift_raw_value: i32,
    rpms_square: &mut [i32],
    transceiver: &mut my_hdlc::HdlcTransceiver,
) {
    let cur_maxes = motor::get_motor_max();

    let mut pwm_to_set: [u16; 4] = [0u16; 4];

    let mut all_zero: bool = true;

    let mut k: usize = 0;
    for x in rpms_square {
        let squared_number: f32 = f32::sqrt(*x as f32);

        let rpm_ratio = squared_number / MAX_RPMS;

        pwm_to_set[k] = (cur_maxes as f32 * rpm_ratio) as u16;

        if pwm_to_set[k] != 0 {
            all_zero = false;
        }
        k += 1;
    }

    // if lift_raw_value < THRESHOLD_LIFT {
    //     for cur_motor_rpm in &mut pwm_to_set {
    //         *cur_motor_rpm = 0;
    //     }
    // } else if !all_zero {
    //     for cur_motor_rpm in &mut pwm_to_set {
    //         *cur_motor_rpm = MIN_PWM.max(*cur_motor_rpm);
    //     }
    // }

    motor::set_motors(pwm_to_set);
}

pub fn actuate_motors_with_rates(
    input_from_controller: &ManualInput,
    my_hdlc: &mut my_hdlc::HdlcTransceiver,
) {
    let Nb: f32 = input_from_controller.get_yaw() as f32 * THRUST_COEFFICIENT;
    let Md: f32 = input_from_controller.get_pitch() as f32 * DRAG_COEFFICIENT;
    let Zd: f32 = -input_from_controller.get_lift() as f32 * DRAG_COEFFICIENT;
    let Ld: f32 = input_from_controller.get_roll() as f32 * DRAG_COEFFICIENT;

    let four_times_bd: f32 = 4.0 * DRAG_COEFFICIENT * THRUST_COEFFICIENT;

    let lift_is_zero: i32 = input_from_controller.get_lift();
    let omega_one: i32 = (((-Nb + (2.0 * Md) - Zd) / (four_times_bd)) as i32).max(0);
    let omega_two: i32 = (((Nb - (2.0 * Ld) - Zd) / (four_times_bd)) as i32).max(0);
    let omega_three: i32 = (((-Nb - (2.0 * Md) - Zd) / (four_times_bd)) as i32).max(0);
    let omega_four: i32 = (((Nb + (2.0 * Ld) - Zd) / (four_times_bd)) as i32).max(0);

    // let to_write = my_hdlc.write_structure(&DeviceCommand::DebugRpms(DebugRpms::new(&[
    //     omega_one,
    //     omega_two,
    //     omega_three,
    //     omega_four,
    // ])));
    // send_bytes(&to_write.0[0..to_write.1]);

    map_rpm_square_to_pwm(
        lift_is_zero,
        &mut [omega_one, omega_two, omega_three, omega_four],
        my_hdlc,
    );
}
