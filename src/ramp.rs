//! # Ramp
//!
//! **Ramp** is an input generator for movements.
//! Generator indicates that it implements an Iterator trait.
//!
//! ## Units of measurement
//!
//! TODO - rethink; start with f32
//!
//! * Directed distance `f32` in meter (or rounds for rotations)
//! * Speed `f32`in meter per seconds (or rounds per second for rotation)
//! * Acceleration `f32`in meter per square second (or rounds per square second
//!   for rotation)
//! * (Directed) distance, speed and acceleration can have positive and negative
//!   values
//!
//! ## Design decisions
//!
//! * Sampling interval is computed, should be small for smooth updates.
//! * Two options:
//! ** Operate with fixed sampling interval
//! ** Operate with dynamic incremental duration detection
//!
//! ## Movement constraints
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
//! # Movement Modes
//!
//! Two modes are possible:
//!
//! * **Speed Mode**: Move at constant speed
//! * **Position Mode**: Move to a specific position and stop moving.
//!
//! ## Position Mode
//!
//! Move as fast as possible from current position to target position.
//! Target position is given as one dimensional parameter, i.e. the signed
//! distance to the target.
//!
//! The movement is accelerated and decelerated to stop at target position.
//! The movement is optimized to reach the target position as fast as
//! possible.
//!
//! ### Example
//!
//! ```rust
//! use sensact::ramp::{Ramp, RampParameter};
//!
//! const MAX_SPEED: f32 = 3.0;
//! const MAX_ACCEL: f32 = 2.0;
//! const MAX_JERK: f32 = 1.0;
//! let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
//! println!("{:?}", rp);
//! let mut r = Ramp::new(rp);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Position Mode: Stop in a distance of - 10
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
//! const MAX_SPEED: f32 = 3.0;
//! const MAX_ACCEL: f32 = 2.0;
//! const MAX_JERK: f32 = 1.0;
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
// TODO test move to position actually stops.

#[derive(Debug)]
pub struct RampParameter {
    max_speed: f32,
    max_acceleration: f32,
    max_jerk: f32,
}

impl RampParameter {
    pub fn new(max_speed: f32, max_acceleration: f32, max_jerk: f32) -> Self {
        Self {
            max_speed,
            max_acceleration,
            max_jerk,
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GeneralizedPosition {
    position: f32,
    speed: f32,
    acceleration: f32,
}

impl GeneralizedPosition {
    pub fn new(position: f32, speed: f32, acceleration: f32) -> Self {
        GeneralizedPosition {
            position,
            speed,
            acceleration,
        }
    }
}

#[derive(Debug)]
pub struct Ramp {
    parameter: RampParameter,
    current: GeneralizedPosition,
    target_distance: Option<f32>,
    target_speed: Option<f32>,
    temp_speed_target: f32, // needed to manage ramp to temp_speed_target
}

const SAMPLE_INTERVAL: f32 = 0.001;
impl Ramp {
    pub fn new(parameter: RampParameter) -> Self {
        Ramp {
            parameter,
            current: Default::default(),
            target_distance: Some(0.0), // do not cause movement as default
            target_speed: None,
            temp_speed_target: 0.0,
        }
    }

    /// Iterator stops if target position is reached
    /// Target position is defined as
    /// acceleration = 0, speed = 0, target position = current position + distance
    pub fn set_target_distance(&mut self, distance: f32) {
        self.target_distance = Some(distance);
        self.target_speed = None;
        // assuming zero speed at the momemnt? - Is that true
        // calculate ramp end speed w/o const speed part
        let mut final_speed = (self.parameter.max_acceleration * distance.abs()).sqrt();
        if final_speed > self.parameter.max_speed {
            final_speed = self.parameter.max_speed;
        }
        self.temp_speed_target = final_speed * distance.signum();
    }

    /// Set speed mode
    ///
    /// * Accelerate to to reach target speed
    /// * Keep moving with a continuous (target) speed
    /// * Iterator runs forever (except target speed is zero)
    ///
    /// # Arguments
    ///
    /// * `speed`: The target speed
    ///    Note: if abs(speed) >  max_speed, target speed gets reduced.
    pub fn set_target_speed(&mut self, speed: f32) {
        let mut s = speed;
        if speed.abs() > self.parameter.max_speed {
            // limit the speed to the maximum possible
            s = self.parameter.max_speed * speed.signum();
        }
        self.target_distance = None;
        self.target_speed = Some(s);
        self.temp_speed_target = s;
    }

    #[allow(dead_code)]
    fn set(&mut self, current: GeneralizedPosition) {
        self.current = current
    }

    pub fn get(&self) -> GeneralizedPosition {
        self.current
    }

    /// Internal function to move to the expected `temp_speed_target`
    ///
    /// # Returns
    ///
    /// * Boolean value that indicates if the speed target is reached
    pub fn move_ramp(&mut self) -> bool {
        // figure out maximum acceleration change i.e. jerk_result
        // it is guaranteed that temp_speed_target is below max_speed
        let dt = SAMPLE_INTERVAL;
        let a_needed = (self.current.speed - self.temp_speed_target) / dt;
        let mut a_result = a_needed;
        // check if new acceleration is smaller than max accelerataion
        if a_needed.abs() > self.parameter.max_acceleration {
            a_result = self.parameter.max_acceleration * a_result.signum();
        }
        // check if acceleration smaller is smaller than max allowed jerk
        let jerk_needed = (self.current.acceleration - a_result) / dt;
        let mut jerk_result = jerk_needed;
        if jerk_needed.abs() > self.parameter.max_jerk {
            jerk_result = self.parameter.max_jerk * jerk_needed.signum();
        }

        // update the positions
        self.current.acceleration += jerk_result * dt;
        self.current.speed += self.current.acceleration * dt;
        self.current.position += self.current.speed * dt;

        // return ramp finished?
        self.temp_speed_target == self.current.speed
    }

    /// Update current generalized position in speed mode
    fn next_speed_mode(&mut self) {
        self.move_ramp();
    }

    /// Update current generalized position in distance mode
    fn next_distance_mode(&mut self) {
        match self.target_distance {
            Some(distance) => {
                let old_pos = self.current.position;
                let ramp_finished_flag = self.move_ramp();
                let new_distance = distance - self.current.position + old_pos;
                self.target_distance = Some(new_distance);
                if ramp_finished_flag {
                    // check if we need to ramp down i.e. set temp_speed_target
                    // to zero
                    let break_distance =
                        self.current.speed.powf(2.0) / self.parameter.max_acceleration;
                    if break_distance >= new_distance {
                        // start breaking by instructing to move_ramp down
                        self.temp_speed_target = 0.0;
                    }
                }
            }
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
    fn next(&mut self) -> Option<Self::Item> {
        if self.target_distance == Some(0.0) {
            // Zero distance to target implies that target position is reached
            return None;
        }
        if self.target_speed == Some(0.0) && self.current.speed == 0.0 {
            // Zero speed implies we do not move anymore
            return None;
        }
        match self.target_speed {
            None => self.next_distance_mode(),
            Some(_speed) => self.next_speed_mode(),
        }
        Some(self.current)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ramp_const_speed_reduce_to_max_speed() {
        const MAX_SPEED: f32 = 3.0;
        const MAX_ACCEL: f32 = 2.0;
        const MAX_JERK: f32 = 1.0;
        let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
        let mut r = Ramp::new(rp);
        r.set_target_speed(-5.0_f32);
        assert_eq!(r.target_speed, Some(-3.0_f32));
        r.set_target_speed(-2.0_f32);
        assert_eq!(r.target_speed, Some(-2.0_f32));
        r.set_target_speed(7.0_f32);
        assert_eq!(r.target_speed, Some(3.0_f32));
        r.set_target_speed(1.0_f32);
        assert_eq!(r.target_speed, Some(1.0_f32));
        r.set_target_speed(0.0_f32);
        assert_eq!(r.target_speed, Some(0.0_f32));
    }

    #[test]
    fn ramp_const_speed_ok() {
        const MAX_SPEED: f32 = 3.0;
        const MAX_ACCEL: f32 = 2.0;
        const MAX_JERK: f32 = 1.0;
        let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
        let mut r = Ramp::new(rp);
        r.set_target_speed(1.0_f32);
        assert_eq!(
            r.next(),
            Some(GeneralizedPosition {
                position: 0.0_f32,
                speed: 0.0_f32,
                acceleration: 0.0_f32,
            })
        );
        assert_eq!(
            r.next(),
            Some(GeneralizedPosition {
                position: 1_f32,
                speed: 1_f32,
                acceleration: 0_f32,
            })
        );
        assert_eq!(
            r.next(),
            Some(GeneralizedPosition {
                position: 2_f32,
                speed: 1_f32,
                acceleration: 0_f32,
            })
        );
    }

    #[test]
    fn ramp_to_position_ok() {
        const MAX_SPEED: f32 = 3.0;
        const MAX_ACCEL: f32 = 2.0;
        const MAX_JERK: f32 = 1.0;
        let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
        let mut r = Ramp::new(rp);
        r.set_target_distance(2_f32);
        assert_eq!(
            r.next(),
            Some(GeneralizedPosition {
                position: 0_f32,
                speed: 0_f32,
                acceleration: 0_f32,
            })
        );
        assert_eq!(
            r.next(),
            Some(GeneralizedPosition {
                position: 2_f32,
                speed: 0_f32,
                acceleration: 0_f32,
            })
        );
        assert_eq!(r.next(), None);
    }
}
