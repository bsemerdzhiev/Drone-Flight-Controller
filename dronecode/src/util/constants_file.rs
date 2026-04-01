use fixed::consts;
use fixed::types::{I16F16, I19F13, I20F12, I24F8, I2F30, I32F32, I3F29, I8F24};

pub type SensorFixedType = I20F12;
pub type RPMFixedType = I20F12;

pub type DegreeType = I20F12;
pub type PIDValuesType = I20F12;
pub type QuaternionValuesType = I20F12;
pub type TimeDifferenceType = I20F12;

pub const PI: QuaternionValuesType = QuaternionValuesType::lit("3.14159265358979");

pub const RAD_TO_DEGREE: DegreeType = DegreeType::lit("57.2957795");
pub const DEGREE_TO_RAD: DegreeType = DegreeType::lit("0.017453");

pub const MAX_LIFT: DegreeType = DegreeType::lit("15");

pub const YAW_RATE: DegreeType = DegreeType::lit("80");
pub const PITCH_DEGREE: DegreeType = DegreeType::lit("20");
pub const ROLL_DEGREE: DegreeType = DegreeType::lit("20");
