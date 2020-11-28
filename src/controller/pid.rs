//! # PID and PID-T1 Controller
//!
//! The PID controller implementation is capable to run als PID controller or
//! as PID-T1 controller (with a t1 delay)
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
//! use steer_and_control::controller::pid::PidController;
//! use steer_and_control::controller::pid_parameter::{PidParameterAdditive};
//!
//! let mut actual = iter::repeat(0.0_f32); // This is what we measure
//! // This composes a step response input as target, i.e. what we want
//! let mut target_pre = iter::repeat(0.0_f32).take(10); // 10 samples before step
//! let mut target_post = iter::repeat(1.0_f32); // samples after the step
//! let mut target = target_pre.chain(target_post);
//!
//! let p = PidParameterAdditive::new(1.0).set_integral(1.0);
//! let mut pid = PidController::<f32>::new(0.000_01).set(p);
//! println!("PidController: {:?}", pid);
//!
//! let output = actual.zip(target)  // provide input tupel (actual, target)
//!     .map(|x| pid.control(x))  // run the controller
//!     .take(1000)  // Stop processing after 1000 samples
//!     // .plot()  // Plot the step response
//!     .collect::<Vec<f32>>();
//! println!("Result {:?}", output);
//! ```
//!
//! ## MCU example
//!
//! On an embedded micro controller integrate the controller in a everlasting
//! loop.
//!
//! The example shows a PID-T1 controller as 32-bit fixed point implementation.
//! It runs as at a sample rate of 20 kHz and the delay is 1 millisecond.
//!
//! ```ignore
//! use std::iter;
//! use steer_and_control::controller::pid::PidController;
//! use steer_and_control::controller::pid_parameter::{PidParameterAdditive};
//!
//! // Sensor that measures actual value - implements an iterator
//! let process = Sensor::new(); // does not compile
//!
//! let setpoint = iter::repeat(500); // Setpoint desired value is injected as iterator
//! let input = process.zip(setpoint);
//!
//! let p = PidParameterAdditive::new(1.0).set_integral(0.2).set_differential(0.4);
//! let mut pid = PidController::<i32>::new_with_t1(0.000_01, 0.000_1).set(p);
//!
//! // Output device - consumes iterator
//! let mut actuator = Actuator::new(); // does not compile
//! let mut output = input.map(|x| pid.control(x));  // configure the controller processing
//! for value in output {
//!     // run ad infinitum
//!     actuator.set(value);
//! }
//! ```

use super::pid_parameter::PidParameterAdditive;
use num_traits::{zero, Zero};

#[derive(Debug)]
pub struct PidController<T: Zero> {
    sample_interval: f32, // in milliseconds
    delay: Option<f32>,   // in milliseconds; only for PID-T1 controller
    error: (T, T, T),     // intermediate error variables (current, last, before_last)
    manipulated: (T, T),  // intermediate manipulated variables (last, before_last)
    a: (T, T),            // pre-calculated control step parameter
    b: (T, T, T),         // a for manipulated (output) b for error (input)
}

const DEFAULT_SAMPLE_INTERVAL: f32 = 0.001; // in seconds
const FIXED_POINT_CORR_I: i64 = 1000; // 1 fixed point equals 0.001
const FIXED_POINT_CORR_F: f32 = FIXED_POINT_CORR_I as f32;
const SAMPLE_INTERVAL_CORR_I: i64 = 100_000; // for integer arithmetics
const SAMPLE_INTERVAL_CORR_F: f32 = SAMPLE_INTERVAL_CORR_I as f32;

impl<T: Zero> Default for PidController<T> {
    fn default() -> Self {
        Self {
            sample_interval: DEFAULT_SAMPLE_INTERVAL,
            delay: None, // PID controller only
            error: (zero(), zero(), zero()),
            manipulated: (zero(), zero()),
            a: (zero(), zero()),
            b: (zero(), zero(), zero()),
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
    /// Return an PID-T1 Controller Instance
    ///
    /// # Arguments
    ///
    /// * `sample_interval` - sample interval is encoded in seconds;
    ///    valid range is 0.000_01 (10 microsecond) to 1.0 (1 second)
    ///    Constructor panics if range is violated
    /// * `delay`- delay time of T1 part of the PID-T1 controller;
    ///    is encoded in seconds; must be greater than sample interval
    ///    typically approximately a magnitude of 10 times sample interval;
    ///    Constructor panics if smaller than sample_interval.
    ///
    pub fn new_with_t1(sample_interval: f32, delay: f32) -> Self {
        let relative = delay / sample_interval;
        if relative < 1.0 {
            panic!("T1 delay needs to be greater than sample interval");
        }
        Self {
            delay: Some(delay),
            ..Self::new(sample_interval)
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
        let k_d: i64 = (parameter.differential * FIXED_POINT_CORR_F) as i64;
        let k_i: i64 = (parameter.integral * FIXED_POINT_CORR_F) as i64;
        let k_p: i64 = (parameter.proportional * FIXED_POINT_CORR_F) as i64;
        let s_ivl = (self.sample_interval * SAMPLE_INTERVAL_CORR_F) as i64;
        let delay = match self.delay {
            Some(d) => (d * SAMPLE_INTERVAL_CORR_F) as i64,
            None => s_ivl,
        };

        Self {
            a: (
                // Zero for PID != Zero for PID-T1
                (FIXED_POINT_CORR_I * s_ivl / delay - FIXED_POINT_CORR_I) as i32,
                // One for PID != Zero for PID-T1
                (2 * FIXED_POINT_CORR_I - FIXED_POINT_CORR_I * s_ivl / delay) as i32,
            ),
            b: (
                ((k_d * SAMPLE_INTERVAL_CORR_I * SAMPLE_INTERVAL_CORR_I
                    - k_p * s_ivl * SAMPLE_INTERVAL_CORR_I
                    + k_i * s_ivl * s_ivl)
                    / delay
                    / SAMPLE_INTERVAL_CORR_I) as i32,
                ((-2 * k_d * SAMPLE_INTERVAL_CORR_I + k_p * s_ivl) / delay) as i32,
                (k_d * SAMPLE_INTERVAL_CORR_I / delay) as i32,
            ),
            ..self
        }
    }

    pub fn control(&mut self, (desire, process): (i32, i32)) -> i32 {
        // update previous error variables
        let error = desire - process;
        self.error = (self.error.1, self.error.2, error);

        let manipulated: i32 = (self.b.2 * self.error.2
            + self.b.1 * self.error.1
            + self.b.0 * self.error.0
            + self.a.1 * self.manipulated.1
            + self.a.0 * self.manipulated.0)
            / FIXED_POINT_CORR_I as i32;

        // update previous manipulated variables
        self.manipulated = (self.manipulated.1, manipulated);

        manipulated
    }
}

impl PidController<f32> {
    pub fn set(self, parameter: PidParameterAdditive) -> Self {
        // TODO remove next line at release build?
        additive_pid_parameter_sanity_check(&parameter);

        let s_ivl = self.sample_interval;
        let delay = match self.delay {
            Some(d) => d,
            None => s_ivl,
        };

        Self {
            a: (
                s_ivl / delay - 1.0, // Zero for PID != Zero for PID-T1
                2.0 - s_ivl / delay, // One for PID != Zero for PID-T1
            ),
            b: (
                (parameter.differential - parameter.proportional * s_ivl
                    + parameter.integral * s_ivl * s_ivl)
                    / delay,
                (-2.0 * parameter.differential + parameter.proportional * s_ivl) / delay,
                parameter.differential / delay,
            ),
            ..self
        }
    }

    pub fn control(&mut self, (desire, process): (f32, f32)) -> f32 {
        // update previous error variables
        let error = desire - process;
        self.error = (self.error.1, self.error.2, error);

        let manipulated: f32 = self.b.2 * self.error.2
            + self.b.1 * self.error.1
            + self.b.0 * self.error.0
            + self.a.1 * self.manipulated.1
            + self.a.0 * self.manipulated.0;

        // update previous manipulated variables
        self.manipulated = (self.manipulated.1, manipulated);

        manipulated
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use float_cmp::approx_eq;

    macro_rules! assert_float_b_coeff_eq {
        ($a: expr, $b:expr, $epsilon: expr) => {
            if !approx_eq!(f32, $a.0, $b.0, epsilon = $epsilon)
                || !approx_eq!(f32, $a.1, $b.1, epsilon = $epsilon)
                || !approx_eq!(f32, $a.2, $b.2, epsilon = $epsilon)
            {
                panic!("assertion failed\n  left:  {:?}\n  right: {:?}", $a, $b);
            }
        };
    }

    macro_rules! approx_int_eq {
        ($a: expr, $b:expr, $epsilon: expr) => {
            (($a - $b).abs() <= $epsilon)
        };
    }

    macro_rules! assert_int_b_coeff_eq {
        ($a: expr, $b:expr, $epsilon: expr) => {
            if !approx_int_eq!($a.0, $b.0, $epsilon)
                || !approx_int_eq!($a.1, $b.1, $epsilon)
                || !approx_int_eq!($a.2, $b.2, $epsilon)
            {
                panic!("assertion failed\n  left:  {:?}\n  right: {:?}", $a, $b);
            }
        };
    }

    #[test]
    fn init_pid_ok() {
        let p = PidParameterAdditive {
            proportional: 2.0,
            integral: 1.0,
            differential: 4.0,
        };
        let pid = PidController::<f32>::new(0.000_05).set(p);
        assert_eq!(pid.a, (0.0, 1.0));
        assert_float_b_coeff_eq!(pid.b, (80_000.0, -160_000.0, 80_000.0), 2.0);
        let pid = PidController::<i32>::new(0.000_05).set(p);
        assert_eq!(pid.a, (0, 1000));
        assert_int_b_coeff_eq!(pid.b, (80_000_000, -160_000_000, 80_000_000), 2_000);
    }

    #[test]
    fn init_pid_t1_ok() {
        let p = PidParameterAdditive {
            proportional: 2.0,
            integral: 1.0,
            differential: 4.0,
        };
        let pid = PidController::<f32>::new_with_t1(0.01, 0.1).set(p);
        assert_eq!(pid.a, (-0.9, 1.9));
        assert_float_b_coeff_eq!(pid.b, (39.8, -79.8, 40.0), 0.001);

        let pid = PidController::<i32>::new_with_t1(0.000_05, 0.000_05).set(p);
        assert_eq!(pid.a, (0, 1000));
        assert_int_b_coeff_eq!(pid.b, (79_998_000, -159_998_000, 80_000_000), 10_000);

        let pid = PidController::<f32>::new_with_t1(0.000_05, 0.000_5).set(p);
        assert_eq!(pid.a, (-0.9, 1.9));
        assert_float_b_coeff_eq!(pid.b, (8000.0, -16_000.0, 8000.0), 1.0);
        let pid = PidController::<i32>::new_with_t1(0.000_05, 0.000_5).set(p);
        assert_eq!(pid.a, (-900, 1900));
        assert_int_b_coeff_eq!(pid.b, (8_000_000, -16_000_000, 8_000_000), 1_000);
    }

    #[test]
    #[should_panic]
    fn pid_new_should_panic_on_sample_interval_too_high() {
        let _pid = PidController::<i32>::new(1.1);
    }

    #[test]
    #[should_panic]
    fn pid_t1_new_should_panic_on_sample_interval_too_high() {
        let _pid = PidController::<i32>::new_with_t1(1.1, 2.2);
    }

    #[test]
    #[should_panic]
    fn pid_t1_new_should_panic_on_delay_too_low() {
        let _pid = PidController::<i32>::new_with_t1(0.5, 0.4);
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
        let mut pid = PidController::<i32>::new(0.001).set(p);
        // double check the parameters
        assert_int_b_coeff_eq!(pid.b, (500_000, -1_000_000, 500_000), 1000);
        // model the step response input to see the controller in action
        assert_eq!(pid.control((0, 0)), 0);
        assert_eq!(pid.control((1000, 0)), 500_000);
        assert_eq!(pid.control((1000, 0)), 500);
        assert_eq!(pid.control((1000, 0)), 500);
    }

    #[test]
    fn control_onf32_ok() {
        let p = PidParameterAdditive::new(0.5).set_differential(0.5);
        let mut pid = PidController::<f32>::new(0.000_01).set(p);
        // double check the parameters
        assert_float_b_coeff_eq!(pid.b, (50_000.0, -100_000.0, 50_000.0), 1.0);
        // model the step response input to see the controller in action
        assert_eq!(pid.control((0.0, 0.0)), 0.0);
        assert_eq!(pid.control((1.0, 0.0)), 50_000.0);
        assert_eq!(pid.control((1.0, 0.0)), 0.5);
    }
}
