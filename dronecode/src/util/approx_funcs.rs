use fixed::{traits::FixedSigned, types::I17F15};

pub fn approx_sqrt<F: FixedSigned>(n: F) -> F {
    if n <= F::ZERO {
        return F::ZERO;
    }
    let mut root: F = F::ZERO;

    let leading = n.leading_zeros() - 1u32;
    let total = (F::FRAC_NBITS + F::INT_NBITS);
    if leading >= total {
        return F::ZERO;
    }

    let mut x = if leading > F::INT_NBITS {
        F::from_num(1) >> ((leading - F::INT_NBITS) / 2)
    } else {
        F::from_num(1) << ((F::INT_NBITS - leading) / 2)
    };

    let mut prev = F::ZERO;
    for _i in 0..5 {
        if x == 0 {
            break;
        }
        root = (x + (n / x)) / F::from_num(2);
        if x == root || root == prev {
            break;
        }
        x = root;
    }
    root
}
