//! # PID Controller
//!
//! ## Requirements
//!
//! * The PID controller should run on MCU (32 bit, w/o FPU)
//! * The PID controller should also run as simulator on any arbitrary host
//!   including plotting of response diagrams
//!
//! ## Design Decision
//!
//! 1. Control loop is implemented as iterator
//!    * Ease of simulator integration
//!    * Ease of integration into ISR
//!    * Iterator end (None returned) allows switch of of actuator and stop
//!      simulation
//! 2. Use of ``no_std`` environment to meet limited MCU capabilities
//! 3. Fixed type on Iterator as i32 and fixed point arithmetics
//!    fast and appropriate for any 32-bit MCU; optimized for ISR's with
//!    minimal operations can go up to 50 kHz ISR triggering rate
//! 4. Fixed and little sampling interval allows to pre-compute processing
//!    constants and speeds up sample processing
//!    * Allows to implementing difference quotients instead of
//!      more computing intensive z-transformation
//!
//! ## Simulation example
//!
//! In an std environment simulate the step response as and plot it
//!
//! ```rust
//! use std::iter;
//! use sensact::pid::{PidController, pid}
//!
//! let mut actual = iter::repeat(0); // This is what we measure
//! // This composes a step response input as target, i.e. what we want
//! let mut target_pre = iter::repeat(0).take(10); // 10 samples before step
//! let mut target_post = iter::repeat(1); // samples after the step
//! let mut target = target_pre.chain(target_post);
//!
//! let p = PidParameter {
//!     proportional: 0.5,
//!     integral: 1.0,
//!     differential: 2.0,
//!     delay: 1000, // 10 millisec time delay
//! };
//! let pid = PidController::new(50); // .05 millisec sample interval
//!
//! let output = actual.zip(target)  // provide input tupel (actual, target)
//!     .map(|(d, p)| pid.control(d, p)))  // run the controller
//!     .take(1000)  // Stop processing after 1000 samples
//!     .plot()  // Plot the step response
//! ```
//!
//! ## MCU example
//!
//! On an embedded micro controller integrate the controller in a everlasting
//! loop
//!
//! ```ignore
//! use sensact::pid::{PidController, pid}
//!
//! let mut actual = Sensor::new(); // Sensor that measures actual value
//!
//! // This composes a step response input as target, i.e. what we want
//! let mut target_pre = iter::repeat(0).take(10); // 10 samples before step
//! let mut target_post = iter::repeat(1); // samples after the step
//! let mut target = target_pre.chain(target_post);
//!
//! let p = PidParameter {
//!     proportional: 0.5,
//!     integral: 1.0,
//!     differential: 2.0,
//!     delay: 1000, // 10 millisec time delay
//! };
//! let pid = PidController::new(50); // .05 millisec sample interval
//!
//! let mut actuator = Actuator::new()  // Output device
//! let values = actual.zip(target)  // provide input tupel (actual, target)
//! .map(|(d, p)| pid.control(d, p)))  // run the controller
//! for value in values {
//!     // run ad infinitum
//!     actuator.set(value);
//! }
//! ```
//!

/// The PID controller with flexible sampling rate
///
/// Manipulated variable (u), Desired value (w), process (sensed) variable (y)
/// are all of the same type t
///
use super::pid_parameter::PidParameterAdditive;
use num_traits::{zero, Zero};

#[derive(Debug)]
pub struct PidController<T: Zero> {
    sample_interval: f32, // in milli seconds
    error: (T, T, T),     // intermediate error variables (current, last, before_last)
    manipulated: T,       // intermediate manipulated variables (last, before_last)
    a: (T, T, T),         // pre-calculated control step parameter
}

const DEFAULT_SAMPLE_INTERVAL: f32 = 0.001; // in seconds
const FIXED_POINT_CORR_I: i32 = 1000; // 1 fixed point equals 0.001
const FIXED_POINT_CORR_F: f32 = FIXED_POINT_CORR_I as f32;
const SAMPLE_INTERVAL_CORR: f32 = 100_000.0;

impl<T: Zero> Default for PidController<T> {
    fn default() -> Self {
        Self {
            sample_interval: DEFAULT_SAMPLE_INTERVAL,
            error: (zero(), zero(), zero()),
            manipulated: zero(),
            a: (zero(), zero(), zero()),
        }
    }
}

impl<T: Zero> PidController<T> {
    /// Return an PID Controller Instance
    ///
    /// # Arguments
    ///
    /// * `sample_interval` - sample interval is encoded in seconds
    ///    valid range is 0.000_01 (10 microsecond) to 1.0 (1 second)
    ///    Constructor panics if range is violated
    ///
    pub fn new(sample_interval: f32) -> Self {
        if sample_interval < 0.000_01 || sample_interval > 1.0 {
            panic!("Sample interval not in [10_us..1_s]");
        }
        Self {
            sample_interval,
            ..Default::default()
        }
    }
}

// Check if the meaningful parameter are injected (conservative programming)
fn additive_pid_parameter_sanity_check(p: &PidParameterAdditive) {
    if p.proportional <= 0.0
        || p.proportional > 10.0
        || p.integral < 0.0
        || p.integral > 10.0
        || p.differential < 0.0
        || p.differential > 10.0
    {
        panic!("PID parameter out of range");
    }
}

impl PidController<i32> {
    pub fn set(self, parameter: PidParameterAdditive) -> Self {
        // TODO remove next line at release build?
        additive_pid_parameter_sanity_check(&parameter);
        let k_d: i32 = (parameter.differential * FIXED_POINT_CORR_F) as i32;
        let k_i: i32 = (parameter.integral * FIXED_POINT_CORR_F) as i32;
        let k_p: i32 = (parameter.proportional * FIXED_POINT_CORR_F) as i32;
        let s_ivl = (self.sample_interval * SAMPLE_INTERVAL_CORR) as i32;

        Self {
            a: (
                k_d / s_ivl,
                -2 * k_d / s_ivl - k_p,
                k_d / s_ivl + k_p + k_i * s_ivl,
            ),
            ..self
        }
    }

    pub fn control(&mut self, (desire, process): (i32, i32)) -> i32 {
        // update previous error variables
        let error = desire - process;
        self.error = (self.error.1, self.error.2, error);

        let manipulated: i32 =
            (self.a.2 * self.error.2 + self.a.1 * self.error.1 + self.a.0 * self.error.0)
                / FIXED_POINT_CORR_I
                + self.manipulated;

        // update previous manipulated variables
        self.manipulated = manipulated;

        manipulated
    }
}

impl PidController<f32> {
    pub fn set(self, parameter: PidParameterAdditive) -> Self {
        // TODO remove next line at release build?
        additive_pid_parameter_sanity_check(&parameter);

        let s_ivl = self.sample_interval * SAMPLE_INTERVAL_CORR;

        Self {
            a: (
                parameter.differential / s_ivl,
                -2.0 * parameter.differential / s_ivl - parameter.proportional,
                parameter.differential / s_ivl
                    + parameter.proportional
                    + parameter.integral * s_ivl,
            ),
            ..self
        }
    }

    pub fn control(&mut self, (desire, process): (f32, f32)) -> f32 {
        // update previous error variables
        let error = desire - process;
        self.error = (self.error.1, self.error.2, error);

        let manipulated: f32 = self.a.2 * self.error.2
            + self.a.1 * self.error.1
            + self.a.0 * self.error.0
            + self.manipulated;

        // update previous manipulated variables
        self.manipulated = manipulated;

        manipulated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_ok() {
        let p = PidParameterAdditive {
            proportional: 2.0,
            integral: 1.0,
            differential: 4.0,
        };
        let pid = PidController::<f32>::new(0.000_05).set(p);
        assert_eq!(
            pid.a,
            (
                4.0 / 5.0,                   // 0.8
                -2.0 * 4.0 / 5.0 - 2.0,      // -3.6
                2.0 + 1.0 * 5.0 + 4.0 / 5.0, // 7.8
            )
        );
        let pid = PidController::<i32>::new(0.000_05).set(p);
        assert_eq!(
            pid.a,
            (4000 / 5, -2 * 4000 / 5 - 2000, 2000 + 1000 * 5 + 4000 / 5,)
        );
    }

    #[test]
    #[should_panic]
    fn pid_new_should_panic_on_sample_interval_too_high() {
        let _pid = PidController::<i32>::new(1.1);
    }

    #[test]
    #[should_panic]
    fn pid_new_should_panic_on_sample_interval_too_low() {
        let _pid = PidController::<i32>::new(0.000_0001);
    }

    #[test]
    #[should_panic]
    fn pid_set_parameter_should_panic_on_proportional_too_high() {
        let p = PidParameterAdditive {
            proportional: 11.0,
            integral: 1.0,
            differential: 3.0,
        };
        let _pid = PidController::<i32>::new(1.0).set(p);
    }

    #[test]
    #[should_panic]
    fn pid_set_parameter_should_panic_on_proportional_too_low() {
        let p = PidParameterAdditive {
            proportional: 0.0,
            integral: 1.0,
            differential: 3.0,
        };
        let _pid = PidController::<f32>::new(1.0).set(p);
    }

    #[test]
    #[should_panic]
    fn pid_set_parameter_should_panic_on_integral_too_low() {
        let p = PidParameterAdditive {
            proportional: 1.0,
            integral: -1.0,
            differential: 3.0,
        };
        let _pid = PidController::<i32>::new(1.0).set(p);
    }

    #[test]
    #[should_panic]
    fn pid_set_parameter_should_panic_on_integral_too_high() {
        let p = PidParameterAdditive {
            proportional: 1.0,
            integral: 11.0,
            differential: 3.0,
        };
        let _pid = PidController::<f32>::new(1.0).set(p);
    }

    #[test]
    #[should_panic]
    fn pid_set_parameter_should_panic_on_differential_too_low() {
        let p = PidParameterAdditive {
            proportional: 1.0,
            integral: 1.0,
            differential: -1.0,
        };
        let _pid = PidController::<i32>::new(1.0).set(p);
    }

    #[test]
    #[should_panic]
    fn pid_set_parameter_should_panic_on_differential_too_high() {
        let p = PidParameterAdditive {
            proportional: 1.0,
            integral: 1.0,
            differential: 11.0,
        };
        let _pid = PidController::<f32>::new(1.0).set(p);
    }

    #[test]
    fn control_oni32_ok() {
        let p = PidParameterAdditive::new(0.5).set_differential(0.5);
        let mut pid = PidController::<i32>::new(0.000_01).set(p);
        // double check the parameters
        assert_eq!(pid.a, (500, -1500, 1000));
        // model the step response input to see the controller in action
        assert_eq!(pid.control((0, 0)), 0);
        assert_eq!(pid.control((1000, 0)), 1000);
        assert_eq!(pid.control((1000, 0)), 500);
        assert_eq!(pid.control((1000, 0)), 500);
    }

    #[test]
    fn control_onf32_ok() {
        let p = PidParameterAdditive::new(0.5).set_differential(0.5);
        let mut pid = PidController::<f32>::new(0.000_01).set(p);
        // double check the parameters
        assert_eq!(pid.a, (0.5, -1.5, 1.0));
        // model the step response input to see the controller in action
        assert_eq!(pid.control((0.0, 0.0)), 0.0);
        assert_eq!(pid.control((1.0, 0.0)), 1.0);
        assert_eq!(pid.control((1.0, 0.0)), 0.5);
        assert_eq!(pid.control((1.0, 0.0)), 0.5);
        assert_eq!(pid.control((1.0, 0.0)), 0.5);
    }
}
