//! Print values of a random sensor
//!

use sensact::sensor;
use sensact::Sensor;

fn main() {
    let s = sensor::UniformDistributionSensor::new(2.0, 5.4);
    println!("Sensor random: {}", s.get());

    println!("Sensor random: {}", sensor::ConstValueSensor::new(2).get());
}
