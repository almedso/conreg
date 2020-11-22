//! # Ramp
//!
//! **Ramp** is an input generator for movement pathes.
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
//! use steer_and_control::steerer::ramp::{Ramp, RampConstraints};
//!
//! let rc: RampConstraints = Default::default().set_speed(1.0);
//! println!("{:?}", rp);
//! let mut r = Ramp::new(0.1, rc);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Position Mode: Stop in a distance of - 10
//! r.set_target_distance(10.0);
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
//! use steer_and_control::steerer::ramp::{Ramp, RampParameter};
//!
//! let rc: RampConstraints = Default::default().set_acceleration(1.0);
//! println!("{:?}", rp);
//! let mut r = Ramp::new(0.1, rc);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Speed Mode: Accelerate to the speed of 2 and keep moving
//! r.set_target_speed(2.0);
//! r.inspect( |val| println!("Iterate: {:?},", val))
//!  .take(10)  // Make sure we stop
//!  .count();  // Consume the iterator
//! ```

// TODO allow for move at constant speed
// TODO test move to position actually stops.

#[derive(Debug, Clone, Copy)]
pub struct RampConstraints {
    max_speed: Option<f32>,
    max_acceleration: Option<f32>,
    max_jerk: Option<f32>,
}

impl Default for RampConstraints {
    /// The default constraints for a ramp are none
    fn default() -> Self {
        Self {
            max_speed: None,
            max_acceleration: None,
            max_jerk: None,
        }
    }
}
impl RampConstraints {
    pub fn max_speed(&self, value: f32) -> Self {
        Self {
            max_speed: Some(value),
            ..*self
        }
    }

    pub fn max_acceleration(&self, value: f32) -> Self {
        Self {
            max_acceleration: Some(value),
            ..*self
        }
    }

    pub fn max_jerk(&self, value: f32) -> Self {
        Self {
            max_jerk: Some(value),
            ..*self
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GeneralizedPosition {
    pub position: f32,
    pub speed: f32,
    pub acceleration: f32,
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

const MAX_5G: f32 = 9.91 * 5.0; // 5G is the super max acceleration to consider at all
const MAX_100M_PER_SEC: f32 = 100.0; // 100 m per seconds as super maximum speed

#[derive(Debug, Clone, Copy)]
pub struct Ramp {
    sample_interval: f32, // measured in secods
    constraints: RampConstraints,
    current: GeneralizedPosition,
    target_distance: Option<f32>,
    target_speed: Option<f32>,
    temp_speed_target: f32, // needed to manage ramp to temp_speed_target
}

impl Ramp {
    /// Ramp Constructor
    ///
    /// # Arguments
    ///
    /// * `sample_interval` - The sample intervall is the time in seconds that is
    ///    used for between two iterator next calls to recompute the new generalized
    ///    position
    /// * `constraints` - Any speed, acceleration or jerk constraints that must
    ///    be considered for path planning
    pub fn new(sample_interval: f32, constraints: RampConstraints) -> Self {
        Ramp {
            sample_interval,
            constraints,
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
        // assuming zero speed at the moment? - Is that true
        // calculate ramp end speed w/o const speed part
        let mut final_speed = (self.consider_max_acceleration(MAX_5G) * distance.abs()).sqrt();
        if final_speed > self.consider_max_speed(MAX_100M_PER_SEC) {
            final_speed = self.consider_max_speed(MAX_100M_PER_SEC);
        }
        self.temp_speed_target = final_speed * distance.signum();
    }

    /// return either speed input of max_speed (sign corrected)
    fn consider_max_speed(&self, speed: f32) -> f32 {
        match self.constraints.max_speed {
            None => speed,
            Some(max) => {
                if speed.abs() > max {
                    max * speed.signum()
                } else {
                    speed
                }
            }
        }
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
        let s = self.consider_max_speed(speed);
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

    // return input acceleration or max acceleration - consider the sign
    fn consider_max_acceleration(&self, acceleration: f32) -> f32 {
        match self.constraints.max_acceleration {
            None => acceleration,
            Some(max) => {
                if acceleration.abs() > max {
                    max * acceleration.signum()
                } else {
                    acceleration
                }
            }
        }
    }
    // return input jerk or max jerk - consider the sign
    fn consider_max_jerk(&self, jerk: f32) -> f32 {
        match self.constraints.max_jerk {
            None => jerk,
            Some(max) => {
                if jerk.abs() > max {
                    max * jerk.signum()
                } else {
                    jerk
                }
            }
        }
    }

    /// Internal function to move to the expected `temp_speed_target`
    ///
    /// # Returns
    ///
    /// * Boolean value that indicates if the speed target is reached
    pub fn move_ramp(&mut self) -> bool {
        // it is guaranteed that temp_speed_target is below max_speed

        const TIME_UNIT: f32 = 1.0; // time unit of 1 second - just for readability
        let a_needed = (self.current.speed - self.temp_speed_target) / TIME_UNIT;
        let a_result = self.consider_max_acceleration(a_needed);

        // check if acceleration is smaller than max allowed jerk
        let jerk_needed = (self.current.acceleration - a_result) / TIME_UNIT;
        let jerk_result = self.consider_max_jerk(jerk_needed);

        // update the generalized position - the sequence matters
        self.current.acceleration += jerk_result * self.sample_interval;
        self.current.speed += self.current.acceleration * self.sample_interval;
        self.current.position += self.current.speed * self.sample_interval;

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
                        self.current.speed.powf(2.0) / self.consider_max_acceleration(MAX_5G);
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
    use float_cmp::approx_eq;

    macro_rules! s_gp {
        ($p:expr, $s:expr, $a:expr) => {
            Some(GeneralizedPosition {
                position: $p,
                speed: $s,
                acceleration: $a,
            })
        };
    }

    macro_rules! assert_general_position_eq {
        ($cond:expr, $expected:expr) => {
            if ! cmp_ge_position($cond, $expected) {
                panic!("assertion failed, different generalized positions:\n  left:  {:?}\n  right: {:?}", $cond, $expected);
            }
        }
    }
    fn cmp_ge_position(a: Option<GeneralizedPosition>, b: Option<GeneralizedPosition>) -> bool {
        match a {
            Some(va) => match b {
                Some(vb) => {
                    return approx_eq!(f32, va.acceleration, vb.acceleration, epsilon = 0.01)
                        && approx_eq!(f32, va.speed, vb.speed, epsilon = 0.000_1)
                        && approx_eq!(f32, va.position, vb.position, epsilon = 0.000_1)
                }
                None => return false,
            },
            None => return false,
        }
    }

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
    fn check_default_position_ok() {
        const MAX_SPEED: f32 = 2.0;
        const MAX_ACCEL: f32 = 9.81;
        const MAX_JERK: f32 = 1000.0;
        let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
        let r = Ramp::new(rp);
        assert_general_position_eq!(Some(r.current), s_gp!(0.0_f32, 0.0_f32, 0.0_f32));
    }

    // TODO - rewrite and update code #[test]
    fn ramp_const_speed_ok() {
        const MAX_SPEED: f32 = 2.0;
        const MAX_ACCEL: f32 = 9.81;
        const MAX_JERK: f32 = 100.0;
        let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
        let mut r = Ramp::new(rp);
        r.set_target_speed(1.0_f32);
        // check speed after 2 second
        for _i in 1..(2.0 / SAMPLE_INTERVAL) as i32 {
            r.next();
        }
        // TODO work out details
        assert_general_position_eq!(r.next(), s_gp!(0.0_f32, 0.0_f32, 2.0_f32));
        assert_general_position_eq!(r.next(), s_gp!(1.0_f32, 1.0_f32, 0.0_f32));
        assert_general_position_eq!(r.next(), s_gp!(2.0_f32, 1.0_f32, 0.0_f32));
    }

    // TODO - rewrite and update code #[test]
    fn ramp_to_position_ok() {
        const MAX_SPEED: f32 = 3.0;
        const MAX_ACCEL: f32 = 10.0;
        const MAX_JERK: f32 = 1000.0;
        let rp = RampParameter::new(MAX_SPEED, MAX_ACCEL, MAX_JERK);
        let mut r = Ramp::new(rp);
        r.set_target_distance(2.0_f32);
        assert_general_position_eq!(r.next(), s_gp!(0.0_f32, 0.0_f32, 2.0_f32));
        assert_general_position_eq!(r.next(), s_gp!(1.0_f32, 1.0_f32, 0.0_f32));
        assert_general_position_eq!(r.next(), s_gp!(2.0_f32, 1.0_f32, 0.0_f32));
        assert_eq!(r.next(), None);
    }
}
