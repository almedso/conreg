//! # Intelligent ramp
//!
//! Two modes are possible:
//!
//! * **Speed Mode**: Move at constant speed
//! * **Position Mode**: Move to a specific position and stop moving.
//!
//! There are ramp parameters that constraint the movement.
//! Those parameters are:
//!
//! * Maximum speed
//! * Maximum acceleration (e.g. for reduction of motor slippage)
//! * Maximum jerk (deviation of acceleration, for prevention e.g. spilling of
//!   liquids, for reduction of force changes to increase lifetime of physical
//!   structures)
//!
//! ## Position Mode
//!
//! Move as fast as possible from current position to target position.
//! Target position is given as one dimensional parameter, i.e. the signed
//! distance to the target.
//!
//! The movement is accelerated and decelerated to stop at target position.
//!
//!
//! ### Example
//!
//! ```rust
//! use sensact::ramp::{Ramp, RampParameter};
//!
//! const MAX_SPEED: isize = 3;
//! const MAX_ACCEL: isize = 2;
//! const MAX_JERK: isize = 1;
//! let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
//! println!("{:?}", rp);
//! let mut r = Ramp::new(rp);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Position Mode: Stop in a distance of -10
//! r.set_target_distance(10);
//! r.inspect( |val| println!("Iterate: {:?},", val))
//!  .take(10)  // Make sure we stop
//!  .count();  // Consume the iterator
//! ```
//!
//! ## Speed Mode
//!
//! Move at constant speed, never stop moving, except target speed is set to
//! Zero.
//!
//! ### Example
//!
//! ```rust
//! use sensact::ramp::{Ramp, RampParameter};
//!
//! const MAX_SPEED: isize = 3;
//! const MAX_ACCEL: isize = 2;
//! const MAX_JERK: isize = 1;
//! let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
//! println!("{:?}", rp);
//! let mut r = Ramp::new(rp);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Speed Mode: Accelerate to the speed of 2 and keep moving
//! r.set_target_speed(2);
//! r.inspect( |val| println!("Iterate: {:?},", val))
//!  .take(10)  // Make sure we stop
//!  .count();  // Consume the iterator
//! ```

// TODO allow for move at constant speed
// TODO double check target is either position or speed --> option
// TODO add time base to Ramp
// TODO test move to position actually stops.

#[derive(Debug)]
pub struct RampParameter {
    max_speed: isize,
    max_acceleration: isize,
    max_jerk: isize,
}

impl RampParameter {
    pub fn new(max_speed: isize, max_acceleration: isize, max_jerk: isize) -> Self {
        Self {
            max_speed,
            max_acceleration,
            max_jerk,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeneralizedPosition {
    position: isize,
    speed: isize,
    acceleration: isize,
}

impl GeneralizedPosition {
    pub fn new(position: isize, speed: isize, acceleration: isize) -> Self {
        GeneralizedPosition {
            position,
            speed,
            acceleration,
        }
    }

    pub fn default() -> Self {
        Self::new(0, 0, 0)
    }
}

#[derive(Debug)]
pub struct Ramp {
    parameter: RampParameter,
    current: GeneralizedPosition,
    target_distance: Option<isize>,
    target_speed: Option<isize>,
}

impl Ramp {
    pub fn new(parameter: RampParameter) -> Self {
        Ramp {
            parameter,
            current: GeneralizedPosition::default(),
            target_distance: Some(0), // do not cause movement as default
            target_speed: None,
        }
    }

    /// Speed mode move with a continuous speed
    /// Iterator stops if target position is reached
    /// Target position is defined as
    /// acceleration = 0, speed = 0, target position = current position + distance
    pub fn set_target_distance(&mut self, distance: isize) {
        self.target_distance = Some(distance);
        self.target_speed = None;
    }

    /// Speed mode move with a continuous speed
    /// Iterator runs forever
    ///
    /// Note: Ramp parameter max_speed, is ignored in this case
    pub fn set_target_speed(&mut self, speed: isize) {
        self.target_distance = None;
        self.target_speed = Some(speed);
    }

    fn set(&mut self, current: GeneralizedPosition) {
        self.current = current
    }

    pub fn get(&self) -> GeneralizedPosition {
        self.current
    }

    /// Update current generalized position in speed mode
    fn next_speed_mode(&mut self) {
        self.current.position += self.current.speed * 1;
    }

    /// Update current generalized position in distance mode
    fn next_distance_mode(&mut self) {
        self.current.speed = self.parameter.max_speed;
        let distance = self.current.speed * 1;
        self.current.position += distance;
        match self.target_distance {
            Some(d) => self.target_distance = Some(d - distance),
            None => panic!("Target distance must be set in distance mode"),
        }
    }
}

impl Iterator for Ramp {
    type Item = GeneralizedPosition;
    /// Iterate delivers generalized positions towards
    /// the target set (move a distance or move at speed x)
    ///
    /// Iterator stops if target position is reached and
    /// speed equals zero.
    /// Iterator never stops if target speed is different
    /// from zero.
    fn next(&mut self) -> Option<GeneralizedPosition> {
        if self.target_distance == Some(0) {
            // Zero distance to target implies that target position is reached
            return None;
        }
        match self.target_speed {
            None => self.next_distance_mode(),
            Some(_d) => self.next_speed_mode(),
        }
        Some(self.current)
    }
}
