// Copyright 2020, almedso GmbH
//
// Licensed under MIT
// <LICENSE-MIT or http://opensource.org/licenses/MIT>

//! Sensor and Actuator module
//!
//! ## Compatibility
//!
//! The `num-traits` crate is tested for rustc 1.8 and greater.
//!
//! ```
//! const RADIUS = 0815.mm;
//! const TICKS_PER_ROUND = 20;
//! let l1 = PositionSensor();
//! let v2 = RotationSensor(TICKS_PER_ROUND).transform(RADIUS);
//! let mut a DriveActuator();
//! let pid_position = Pos
//!
//! // iterator way
//! let mut output = v2.iter().map(|x| -> pid.control(x, target_val)).collect()
//! l1.iter().map(|speed| -> pid(transform(speed), output).steer(a);
//!
//! loop () {
//!     // iterator way
//!     a.set(pid.next())
//! }
//!

#![no_std]
#[cfg(feature = "std")]
extern crate std;

#[cfg(feature = "std")]
pub mod sensor;

use num_traits::NumOps;

pub trait Sensor<T: PartialEq + NumOps> {
    fn get(&self) -> T;
}

pub trait TimeSourceIterator {
    type MicroSeconds;

    fn next(&mut self) -> Option<Self::MicroSeconds>;
}

#[cfg(feature = "std")]
pub mod timebase;
