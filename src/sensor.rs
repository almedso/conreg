//! Sensors that simulate some measurements

use crate::Sensor;
use num_traits::NumOps;

pub struct ConstValueSensor<T> {
    value: T,
}

impl<T> ConstValueSensor<T> {
    pub fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T: PartialEq + NumOps + Clone> Sensor<T> for ConstValueSensor<T> {
    fn get(&self) -> T {
        return self.value.clone();
    }
}

pub struct UniformDistributionSensor<T> {
    min: T,
    max: T,
}

impl<T> UniformDistributionSensor<T> {
    pub fn new(min: T, max: T) -> Self {
        Self { min, max }
    }
}

use rand::distributions::uniform::SampleUniform;
use rand::Rng;

impl<T: PartialEq + NumOps + SampleUniform + Copy> Sensor<T> for UniformDistributionSensor<T> {
    fn get(&self) -> T {
        let mut rng = rand::thread_rng();
        rng.gen_range(self.min, self.max)
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

    #[test]
    fn uniform_distribution_sensor_discrete_ok() {
        let m = UniformDistributionSensor::new(2, 5).get();
        assert!(2 <= m && m <= 5);
    }

    #[test]
    fn uniform_distribution_sensor_float_ok() {
        let m = UniformDistributionSensor::new(2.0, 5.9).get();
        assert!(2.0 <= m && m <= 5.9);
    }
}
