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
    let sqrt_tol = F::from_num(0.001f32);
    let mut x = n;
    let mut root: F;
    loop {
        root = (x + (n / x)) / F::from_num(2);
        if (x - root).abs() < sqrt_tol {
            break;
        }
        x = root;
    }
    root
}
