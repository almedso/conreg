//! Sensors that simulate some measurements

use crate::Sensor;
use num_traits::NumOps;

pub struct ConstValueSensor<T> {
    value: T,
}

impl<T> ConstValueSensor<T> {
    fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T: PartialEq + NumOps + Clone> Sensor<T> for ConstValueSensor<T> {
    fn get(&self) -> T {
        return self.value.clone();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn const_sensor_ok() {
        assert_eq!(ConstValueSensor::new(5).get(), 5);
        assert_eq!(ConstValueSensor::new(5.1).get(), 5.1);
    }
}
