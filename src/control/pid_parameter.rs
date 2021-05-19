//! Parameter Variants for a PID and PID-T1 controller
//!
use core::convert::From;

/// K_p, K_i, K_d Parameter for a PID and PID-T1 controller
///
/// Additive Form is composed of addition of the P, I and D channel
///
/// u(t) = k_p e(t) + k_i *integral_0^t( e(t)) * k_d * d/dt ( e(t)))
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidParameterAdditive {
    /// Unit: One;  A positive number (> 0.0) i.e. there is always a D portion
    pub proportional: f32,
    /// Unit: One per second; A none-negative number (>= 0.0); Zero means no I portion
    pub integral: f32,
    /// Unit: One times second; A none-negative number (>= 0.0); Zero means no D portion
    pub differential: f32,
}

impl PidParameterAdditive {
    /// Constructor for Pid Additive Parameter
    ///
    /// I portion and D portion are set to zero
    ///
    /// # Arguments
    ///
    /// * `proportional` - P portion amplification; must be greater than zero
    pub fn new(proportional: f32) -> Self {
        if proportional <= 0.0 {
            panic!("Positive proportional coefficient required");
        }
        Self {
            proportional,
            integral: 0.0,
            differential: 0.0,
        }
    }

    /// Activate the integral portion of the controller
    pub fn set_integral(&self, integral: f32) -> Self {
        if integral <= 0.0 {
            panic!("Positive integral coefficient required");
        }
        Self { integral, ..*self }
    }

    /// Activate the differential portion of the controller
    pub fn set_differential(&self, differential: f32) -> Self {
        if differential <= 0.0 {
            panic!("Positive differential coefficient required");
        }
        Self {
            differential,
            ..*self
        }
    }
}

/// K_r, t_i, t_d Parameter for a PID and PID-T1 controller
///
/// The Standard Form is considering times for D and I portion
/// and an ultimate amplification for the entire controller
///
/// u(t) = k_p ( e(t) + 1 / t_i *integral_0^t( e(t)) * t_d * d/dt ( e(t)))
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidParameterStandard {
    /// Controller amplification; Unit: One; A Positive Number (> 0.0 )
    pub amplification: f32,
    /// Unit: seconds; A Positive Number (> 0.0 ) Is None if there is no I Portion
    pub integral_time: Option<f32>,
    /// Unit: second; A none-negative number (>= 0.0); Zero means no D portion
    pub differential_time: f32,
}

impl PidParameterStandard {
    /// Constructor for Pid Standard Parameter
    ///
    /// I portion and D portion are set to zero
    ///
    /// # Arguments
    ///
    /// * `amplification` - P portion amplification; must be greater zero
    pub fn new(amplification: f32) -> Self {
        if amplification <= 0.0 {
            panic!("Positive amplification required");
        }
        Self {
            amplification,
            integral_time: None,
            differential_time: 0.0,
        }
    }

    /// Activate the integral portion of the controller
    pub fn set_integral_time(&self, integral_time: f32) -> Self {
        if integral_time <= 0.0 {
            panic!("Positive integral time required");
        }
        Self {
            integral_time: Some(integral_time),
            ..*self
        }
    }

    /// Activate the differential portion of the controller
    pub fn set_differential_time(&self, differential_time: f32) -> Self {
        if differential_time <= 0.0 {
            panic!("Positive differential time required");
        }
        Self {
            differential_time,
            ..*self
        }
    }
}

impl From<PidParameterAdditive> for PidParameterStandard {
    fn from(item: PidParameterAdditive) -> Self {
        Self {
            amplification: item.proportional,
            integral_time: if item.integral == 0.0 {
                None
            } else {
                Some(item.proportional / item.integral)
            },
            differential_time: item.differential / item.proportional,
        }
    }
}

impl From<PidParameterStandard> for PidParameterAdditive {
    fn from(item: PidParameterStandard) -> Self {
        Self {
            proportional: item.amplification,
            integral: match item.integral_time {
                Some(time) => item.amplification / time,
                None => 0.0,
            },
            differential: item.differential_time * item.amplification,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[should_panic]
    fn new_standard_should_panic_on_zero() {
        PidParameterStandard::new(0.0);
    }

    #[test]
    #[should_panic]
    fn new_standard_should_panic_on_negative() {
        PidParameterStandard::new(-1.0);
    }

    #[test]
    #[should_panic]
    fn set_integral_time_should_panic_on_negative() {
        PidParameterStandard::new(1.0).set_integral_time(-1.0);
    }

    #[test]
    #[should_panic]
    fn set_integral_time_should_panic_on_zero() {
        PidParameterStandard::new(1.0).set_integral_time(0.0);
    }

    #[test]
    #[should_panic]
    fn set_differential_time_should_panic_on_negative() {
        PidParameterStandard::new(1.0).set_differential_time(-1.0);
    }

    #[test]
    #[should_panic]
    fn set_differential_time_should_panic_on_zero() {
        PidParameterStandard::new(1.0).set_differential_time(0.0);
    }

    #[test]
    #[should_panic]
    fn new_additive_should_panic_on_zero() {
        PidParameterAdditive::new(0.0);
    }

    #[test]
    #[should_panic]
    fn new_additive_should_panic_on_negative() {
        PidParameterAdditive::new(-1.0);
    }

    #[test]
    #[should_panic]
    fn set_integral_should_panic_on_negative() {
        PidParameterAdditive::new(1.0).set_integral(-1.0);
    }

    #[test]
    #[should_panic]
    fn set_integral_should_panic_on_zero() {
        PidParameterAdditive::new(1.0).set_integral(0.0);
    }

    #[test]
    #[should_panic]
    fn set_differential_should_panic_on_negative() {
        PidParameterAdditive::new(1.0).set_differential(-1.0);
    }

    #[test]
    #[should_panic]
    fn set_differential_should_panic_on_zero() {
        PidParameterAdditive::new(1.0).set_differential(0.0);
    }

    #[test]
    fn convert_standard_to_additive_ok() {
        let s = PidParameterStandard::new(2.0)
            .set_differential_time(1.0)
            .set_integral_time(0.5);
        let a = PidParameterAdditive::new(2.0)
            .set_differential(2.0)
            .set_integral(4.0);
        assert_eq!(a, s.into());

        let s = PidParameterStandard::new(1.0).set_integral_time(0.25);
        let a = PidParameterAdditive::new(1.0).set_integral(4.0);
        assert_eq!(a, s.into());

        let s = PidParameterStandard::new(1.0).set_differential_time(2.0);
        let a = PidParameterAdditive::new(1.0).set_differential(2.0);
        assert_eq!(a, s.into());

        let s = PidParameterStandard::new(1.0);
        let a = PidParameterAdditive::new(1.0);
        assert_eq!(a, s.into());
    }

    #[test]
    fn convert_additive_to_standard_ok() {
        let s = PidParameterStandard::new(2.0)
            .set_differential_time(1.0)
            .set_integral_time(0.5);
        let a = PidParameterAdditive::new(2.0)
            .set_differential(2.0)
            .set_integral(4.0);
        assert_eq!(s, a.into());

        let s = PidParameterStandard::new(1.0).set_integral_time(0.25);
        let a = PidParameterAdditive::new(1.0).set_integral(4.0);
        assert_eq!(s, a.into());

        let s = PidParameterStandard::new(1.0).set_differential_time(2.0);
        let a = PidParameterAdditive::new(1.0).set_differential(2.0);
        assert_eq!(s, a.into());

        let s = PidParameterStandard::new(1.0);
        let a = PidParameterAdditive::new(1.0);
        assert_eq!(s, a.into());
    }
}
