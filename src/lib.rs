// Copyright 2020, almedso GmbH
//
// Licensed under MIT
// <LICENSE-MIT or http://opensource.org/licenses/MIT>

//! Control and regulation module
//!
//! ## Compatibility
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

pub mod control;
pub mod regulation;

pub mod target;
pub mod timebase;
