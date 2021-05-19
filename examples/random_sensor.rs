//! Print values of a random sensor
//!

use conreg::sensor::UniformDistributionSensor;
use conreg::Sensor; // trait must be brought into namespace as well

fn main() {
    let s = UniformDistributionSensor::new(2.0, 5.4);
    println!("Sensor random: {}", s.get());

    // println!("Sensor random: {}", sensor::ConstValueSensor::new(2).get());
}
