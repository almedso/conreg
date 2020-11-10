//! PID controller implemented as iterator
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
//! const PROPORTIONAL: isize = 100;
//! const INTEGRAL: isize = 5;
//! const DIFFERENTIAL: isize = 0;
//! let pid_parameter = PidController::new(PROPORTIONAL, INTEGRAL, DIFFERENTIAL);
//!
//! let output = actual.zip(target)  // provide input tupel (actual, target)
//!     .pid(pid_parameter))  // run the controller
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
//! const PROPORTIONAL: isize = 100;
//! const INTEGRAL: isize = 5;
//! const DIFFERENTIAL: isize = 0;
//! let pid_parameter = PidController::new(PROPORTIONAL, INTEGRAL, DIFFERENTIAL);
//!
//! let mut actuator = Actuator::new()  // Output device
//! let values = actual.zip(target)  // provide input tupel (actual, target)
//!     .pid(pid_parameter))  // run the controller
//! for value in values {
//!     // run ad infinitum
//!     actuator.set(value);
//! }
//! ```
//!


use num_traits::NumOps;
use crate::timebase::TimeDelta;

const FIXED_BASE:isize = 1000;

pub struct PidController<T: NumOps> {
    p_parameter: isize,  // fixed point 3 digits after point
    i_parameter: isize,  // fixed point 3 digits after point
    d_parameter: isize,  // fixed point 3 digits after point
    time_delta: TimeDelta, // delivers time differences
    last: T,  // the last value - TODO check if that is correct
}

impl <T: NumOps> PidController<T> {
    pub fn new(p_parameter, i_parameter, d_parameter, time_delta) -> Self {
        Self {p_parameter, i_parameter, d_parameter, time_delta, last: 0 }
    }

    pub fn control( (target, actual): (T, T) ) -> T {
        let delta_t = time_delta.diff();
        let diff = target - actual;
        self.last += diff * self.p_parameter / FIXED_BASE
            + diff * self.i_parameter * delta_t / FIXED_BASE
            + diff * self.d_parameter / delta_t / FIXED_BASE;
        self.last
    }
}