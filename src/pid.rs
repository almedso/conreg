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

/// Parameter for a PID-T1 controller
///
///
pub struct PidParameter {
    proportional: f32,
    integral: f32,
    differential: f32,
    delay: i32, // in micro seconds (for D-T1 portion)
}

/// The PID-T1 controller with flexible sampling rate
///
/// Manipulated variable (u), Desired value (w), process (sensed) variable (y)
/// are all of the same type t
pub struct PidController {
    sample_interval: i32,
    error: (i32, i32, i32), // intermediate error variables (current, last, before_last)
    manipulated: (i32, i32), // intermediate manipulated variables (last, before_last)
    a: (i32, i32),          // pre-calculated control step parameter
    b: (i32, i32, i32),     // pre-calculated control step parameter
}

const DEFAULT_SAMPLE_INTERVAL: i32 = 100; // equals 1 millisecond
const FIXED_POINT_CORRECTION: i32 = 1000; // 1 fixed point equals 0.001
impl Default for PidController {
    fn default() -> Self {
        Self {
            sample_interval: DEFAULT_SAMPLE_INTERVAL,
            error: (0, 0, 0),
            manipulated: (0, 0),
            a: (0, 0),
            b: (0, 0, 0),
        }
    }
}

impl PidController {
    /// Return an PID Controller Instance
    ///
    /// # Arguments
    ///
    /// * `sample_interval` - sample interval is encoded in 10^-5 seconds
    ///    valid range is 10 microsecond to 1 second
    ///    Constructor panics if range is violated
    ///
    pub fn new(sample_interval: i32) -> Self {
        if sample_interval < 1 || sample_interval > 100_000 {
            panic!("Sample interval not in [10_us..1_s]");
        }
        Self {
            sample_interval,
            ..Default::default()
        }
    }

    pub fn set(self, parameter: PidParameter) -> Self {
        let k_d: i32 = (parameter.differential * 1000.0) as i32;
        let k_i: i32 = (parameter.integral * 1000.0) as i32;
        let k_p: i32 = (parameter.proportional * 1000.0) as i32;

        Self {
            a: (
                FIXED_POINT_CORRECTION
                    - FIXED_POINT_CORRECTION * self.sample_interval / parameter.delay,
                -2 * FIXED_POINT_CORRECTION
                    + FIXED_POINT_CORRECTION * self.sample_interval / parameter.delay,
            ),
            b: (
                (k_d - k_p * self.sample_interval
                    // danger of overflow
                    + k_i * self.sample_interval * self.sample_interval)
                    / parameter.delay,
                -2 * k_d + k_p * self.sample_interval / parameter.delay,
                k_d / parameter.delay,
            ),
            ..self
        }
    }

    pub fn control(&mut self, (desire, process): (i32, i32)) -> i32 {
        // update previous error variables
        let error = desire - process;
        self.error = (self.error.1, self.error.2, error);

        let manipulated: i32 = self.b.2 * self.error.2 / FIXED_POINT_CORRECTION
            + self.b.1 * self.error.1 / FIXED_POINT_CORRECTION
            + self.b.0 * self.error.0 / FIXED_POINT_CORRECTION
            + self.a.1 * self.manipulated.1 / FIXED_POINT_CORRECTION
            + self.a.0 * self.manipulated.0 / FIXED_POINT_CORRECTION;

        // update previous manipulated variables
        self.manipulated = (self.manipulated.1, manipulated);

        manipulated
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_ok() {
        let p = PidParameter {
            proportional: 0.5,
            integral: 1.0,
            differential: 2.0,
            delay: 1000, // 10 millisec time delay
        };
        let pid = PidController::new(50); // .05 millisec sample interval
        let pid = pid.set(p);
        assert_eq!(pid.a, (950, -1950));
        assert_eq!(pid.b, (2477, -3975, 2));
    }

    #[test]
    fn control_ok() {
        let p = PidParameter {
            proportional: 0.5,
            integral: 1.0,
            differential: 2.0,
            delay: 1000, // 10 millisec time delay
        };
        let pid = PidController::new(50); // .05 millisec sample interval
        let mut pid = pid.set(p);
        // model the step response input to see the controller in action
        assert_eq!(pid.control((0, 0)), 0);
        assert_eq!(pid.control((1000, 0)), 2);
        assert_eq!(pid.control((1000, 0)), -3976);
    }
}
