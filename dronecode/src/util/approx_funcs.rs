use fixed::{traits::FixedSigned, types::I17F15};

pub fn approx_sqrt<F: FixedSigned>(n: F) -> F {
    if n == F::ZERO {
        return F::ZERO;
    }
    let mut x = n;
    let mut root: F = F::ZERO;
    let mut prev = F::ZERO;
    for _i in 0..9 {
        root = (x + (n / x)) / F::from_num(2);
        if x == root || root == prev {
            break;
        }
        x = root;
    }
    root
}

pub fn approx_sqrt_rpm<F: FixedSigned>(n: F) -> F {
    if n == F::ZERO {
        return F::ZERO;
    }
    let mut x: F = F::from_num(7350);
    let mut root: F = F::ZERO;
    let mut prev = F::ZERO;
    for _i in 0..5 {
        root = (x + (n / x)) / F::from_num(2);
        if x == root || root == prev {
            break;
        }
        x = root;
    }
    root
}
const PI: I17F15 = I17F15::lit("3.14");

pub fn atan2_approx<F: FixedSigned>(y: F, x: F) -> F {
    let pi = F::from_num(3.14f32);
    if x == F::ZERO {
        return y.signum() * pi / F::from_num(2);
    }
    let angle = y.abs() / x.abs();
    let base = angle - (angle * angle * angle) / F::from_num(3);
    if x > F::ZERO {
        y.signum() * base
    } else {
        y.signum() * (pi - base)
    }
}

const ATAN_TABLE: [I17F15; 16] = [
    I17F15::lit("0.785398163"), // atan(2^0)
    I17F15::lit("0.463647609"), // atan(2^-1)
    I17F15::lit("0.244978663"), // atan(2^-2)
    I17F15::lit("0.124354995"), // atan(2^-3)
    I17F15::lit("0.062418810"), // atan(2^-4)
    I17F15::lit("0.031239833"), // atan(2^-5)
    I17F15::lit("0.015623729"), // atan(2^-6)
    I17F15::lit("0.007812341"), // atan(2^-7)
    I17F15::lit("0.003906230"), // atan(2^-8)
    I17F15::lit("0.001953123"), // atan(2^-9)
    I17F15::lit("0.000976562"), // atan(2^-10)
    I17F15::lit("0.000488281"), // atan(2^-11)
    I17F15::lit("0.000244141"), // atan(2^-12)
    I17F15::lit("0.000122070"), // atan(2^-13)
    I17F15::lit("0.0000610"),   // atan(2^-14)
    I17F15::lit("0.0000305"),   // atan(2^-15)
];

pub fn atan2_cordic<F: FixedSigned>(y_in: F, x_in: F) -> F {
    let mut x: F;
    let mut y: F;
    let angle_offset: F;
    if x_in.is_positive() || x_in.is_zero() {
        x = x_in;
        y = y_in;
        angle_offset = F::ZERO;
    } else {
        let y_sgn = if y_in.is_negative() {
            F::from_num(-1)
        } else {
            F::from_num(1)
        };
        x = -x_in;
        y = -y_in;
        angle_offset = y_sgn * F::from_num(PI);
    }

    let mut z = F::ZERO;
    for i in 0..10usize {
        let y_sgn = if y.is_negative() {
            F::from_num(-1)
        } else {
            F::from_num(1)
        };
        let x_new = x + (y_sgn * (y >> i as u32));
        let y_new = y - (y_sgn * (x >> i as u32));
        let dz = y_sgn * F::from_num(ATAN_TABLE[i]);
        x = x_new;
        y = y_new;
        z += dz;
    }
    z + angle_offset
}
