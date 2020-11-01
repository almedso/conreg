// Copyright 2020, almedso GmbH
//
// Licensed under MIT
// <LICENSE-MIT or http://opensource.org/licenses/MIT>

//! Sensor and Actuator module
//!
//! ## Compatibility
//!
//! The `num-traits` crate is tested for rustc 1.8 and greater.

#![no_std]
#[cfg(feature = "std")]
extern crate std;

// #[cfg(feature = "std")]
pub mod sensor;

use num_traits::NumOps;

pub trait Sensor<T: PartialEq + NumOps> {
    fn get(&self) -> T;
}
