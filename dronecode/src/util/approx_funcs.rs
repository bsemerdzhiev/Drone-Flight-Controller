use fixed::traits::FixedSigned;

pub fn approx_atan2<F: FixedSigned>(y: F, x: F) -> F {
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

pub fn approx_sqrt<F: FixedSigned>(n: F) -> F {
    if n == F::ZERO {
        return F::ZERO;
    }
    let mut x = n;
    let mut root: F = F::ZERO;
    let mut prev = F::ZERO;
    for _i in 0..3 {
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
