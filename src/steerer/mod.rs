//! Steerers
//!
//! Steerer act on some real hardware without any use of sensor information.
//!
//! Steerer implement a strategy, where a view parameter are mapped
//! to an iterator that delivers a sequence of steering data that can directly
//! applied to an actuator.
//!
//! A steerer can be commanded to provide continuous steering input.
//!
pub mod ramp;
