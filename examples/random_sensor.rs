//! Print values of a random sensor
//!

use steer_and_control::sensor::UniformDistributionSensor;
use steer_and_control::Sensor; // trait must be brought into namespace as well

fn main() {
    let s = UniformDistributionSensor::new(2.0, 5.4);
    println!("Sensor random: {}", s.get());

    // println!("Sensor random: {}", sensor::ConstValueSensor::new(2).get());
}
