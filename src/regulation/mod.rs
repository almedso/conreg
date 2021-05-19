//! Regulation
//!
//! Regulation act on some real hardware without any use of sensor information.
//! There is no feedback loop.
//!
//! Regulators implement a strategy, where one to few parameters are mapped
//! to an iterator that delivers a sequence of output data that can directly
//! applied to an actuator.
//!
pub mod ramp;
