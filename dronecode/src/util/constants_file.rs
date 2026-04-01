use fixed::consts;
use fixed::types::{I16F16, I17F15, I24F8};

pub type ChosenFixedPointType = I24F8;

pub const PI: ChosenFixedPointType = ChosenFixedPointType::lit("3.14159265358979");
pub const RAD_TO_DEGREE: ChosenFixedPointType = ChosenFixedPointType::lit("57.29577951");
pub const DEGREE_TO_RAD: ChosenFixedPointType = ChosenFixedPointType::lit("0.01745329251");

pub const MAX_LIFT: ChosenFixedPointType = ChosenFixedPointType::lit("15");

pub const YAW_RATE: ChosenFixedPointType = ChosenFixedPointType::lit("80");
pub const PITCH_DEGREE: ChosenFixedPointType = ChosenFixedPointType::lit("20");
pub const ROLL_DEGREE: ChosenFixedPointType = ChosenFixedPointType::lit("20");
