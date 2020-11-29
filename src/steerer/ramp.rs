//! # Ramp
//!
//! **Ramp** is an input generator for movement paths.
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
//! let rc = RampConstraints::default().max_speed(1.0);
//! println!("{:?}", rc);
//! let mut r = Ramp::new(0.1, rc);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Position Mode: Stop in a distance of - 10
//! r.set_target_relative_position(10.0);
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
//! use steer_and_control::steerer::ramp::{Ramp, RampConstraints};
//!
//! let rc =  RampConstraints::default().max_acceleration(1.0);
//! println!("{:?}", rc);
//! let mut r = Ramp::new(0.1, rc);
//! println!("Current generalized position{:?}", r.get());
//!
//! // Speed Mode: Accelerate to the speed of 2 and keep moving
//! r.set_target_speed(2.0);
//! r.inspect( |val| println!("Iterate: {:?},", val))
//!  .take(10)  // Make sure we stop
//!  .count();  // Consume the iterator
//! ```

/// Ramp constraints are physical limitation of the item to be moved by a ramp
/// The constraints are:
///
/// * speed limit
/// * acceleration limit
/// * jerk limit (deviation of acceleration)
///
/// Jerk limits reduces changing force to the item in question and increases
/// lifetime of the item as well as it prevents spilling in case liquides are
/// moved in open carriers
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
        if value < 0.0 {
            panic!("Negative value is not allowed");
        }
        Self {
            max_speed: Some(value),
            ..*self
        }
    }

    pub fn max_acceleration(&self, value: f32) -> Self {
        if value < 0.0 {
            panic!("Negative value is not allowed");
        }
        Self {
            max_acceleration: Some(value),
            ..*self
        }
    }

    pub fn max_jerk(&self, value: f32) -> Self {
        if value < 0.0 {
            panic!("Negative value is not allowed");
        }
        Self {
            max_jerk: Some(value),
            ..*self
        }
    }
}

/// One dimensional coordinates that consider position, speed and acceleration
/// Represents the kinematic state of the item to be moved by a ramp.
#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub struct GeneralizedPosition {
    pub position: f32,
    pub speed: f32,
    pub acceleration: f32,
}

impl GeneralizedPosition {
    /// Generate a new generalized position - allow to set the position
    /// speed and acceleration are set to Default (0.0)
    pub fn new(position: f32) -> Self {
        GeneralizedPosition {
            position,
            ..Default::default()
        }
    }
}

/// The ramp is defined by sequence of up to 8 steps this function is called
/// when a step has completed and the next step is due to be the current
/// Current step is the state at index zero. next is the one at index 1
/// Steps are represented by RampState data type
#[derive(Debug, Clone, Copy, PartialEq)]
enum RampState {
    AccelerateTo(f32),             // Target acceleration
    AccelerateConst(f32),          // Time to keep the acceleration constant
    SpeedConst(f32),               // Time to keep the constant speed
    SpeedKeep,                     //  The speed to keep (if moving with constant speed)
    SpeedZero,                     // at the end of the movement
    RecomputeToSpeed(f32),         // postponed path planning after reaching any const speed
    RecomputeToPosition(f32, f32), // postponed path planning after reaching zero const speed with position counting
}

#[derive(Debug, Clone, Copy)]
pub struct Ramp {
    sample_interval: f32, // measured in seconds
    constraints: RampConstraints,
    current: GeneralizedPosition,
    state: [RampState; 8], // not more than 8 steps are required to run a ramp
}

// Arbitrary constant e.g. for utmost upper limits
const TIME_UNIT: f32 = 1.0; // time unit of 1 second - just for readability
const MAX_JERK: f32 = 1000_000.0; // that is more than  100_000 g / sec
const MAX_ACCELERATION: f32 = 100_000.0; // that is more than 10_000 g
const MAX_SPEED: f32 = 300_000_000.0; // about light speed in m per sec

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
            state: [RampState::SpeedZero; 8],
        }
    }

    /// Reset the path state to just be stopped as a safe fallback
    fn reset_path_states(&mut self) {
        self.state = [RampState::SpeedZero; 8];
    }

    /// Analyse mandatory step of acceleration changes to determine
    ///
    /// * acceleration to end up (a_opt)
    /// * and new delta_position to move to reduced by the (signed) distance
    ///   spend while changing acceleration
    ///
    /// At this step the formula holds: the da/dt = jerk is constant.
    /// jerk is equal the jerk limit.
    ///
    /// It is assured that the the (potential) acceleration limit is not
    /// exceeded.
    ///
    /// It is assured that the resulting acceleration will not end up
    /// a speed that exceeds the speed limit.
    ///
    /// # Returns
    ///
    /// tuple (a_opt, delta_position )
    /// note about the returned tuple:
    ///    delta_position.abs() >= delta_position.return.abs() >= 0.0
    ///    delta_poosiiton.signum() == a_opt.signum()
    ///
    fn compute_parameters_of_acceleration_change_steps(&self, delta_position: f32) -> (f32, f32) {
        let jerk = self.consider_max_jerk(MAX_JERK);
        // 4 times unconstraint jerked acceleration to a_opt
        // s = j^3 / 3 * t^3 and a_opt =  j * t
        let mut a_opt = (3.0 / 4.0 * jerk.powi(2) * delta_position.abs()).cbrt();

        let a_max_by_accl = self.consider_max_acceleration(MAX_ACCELERATION);
        if a_opt > a_max_by_accl {
            // reduce max acceleration due to acceleration limit
            a_opt = a_max_by_accl;
        }

        let v_max = self.consider_max_speed(MAX_SPEED);
        // we accelerate and decelerate so it is TWO times:  2.0 / TWO = 1.0
        let a_max_by_speed = (v_max * jerk).sqrt();
        if a_opt > a_max_by_speed {
            // reduce max acceleration due to speed limit
            a_opt = a_max_by_speed;
        };
        a_opt *= delta_position.signum();
        (
            // resulting optimal acceleration (with sign)
            a_opt,
            // signed distance to move without changing acceleration portions
            delta_position - a_opt.powi(3) / (3.0 / 4.0 * jerk.powi(2)),
        )
    }

    // Internal function that plans
    //  * path acceleration (a_opt),
    //  * constant acceleration time (t_accl)
    //  * and constant speed time (t_v)
    // under the condition that the prerequisites are met
    // (speed zero, acceleration zero of current generalized position)
    fn plan_for_position(&self, delta_position: f32) -> (f32, f32, f32) {
        let (a_opt, delta_position) =
            self.compute_parameters_of_acceleration_change_steps(delta_position);

        // a_opt is below  acceleration limits, v_res is below speed limit
        // we have a distance of d_p go move
        let jerk = self.consider_max_jerk(MAX_JERK);
        let v_max = self.consider_max_speed(MAX_SPEED);
        let t_v: f32;
        let t_accl: f32;
        // speed gain v_res achieved by TWO accelerate change steps
        let v_res = a_opt.powi(2) / jerk;
        // s_accl is the distance is potentially moved at constant acceleration
        // this step happens TWO times
        let s_accl = (v_max - v_res).powi(2) / a_opt.abs();
        if s_accl >= delta_position.abs() {
            // no need to move at const speed; accelerate 2 times
            t_accl = (delta_position / a_opt).abs().sqrt();
            t_v = 0.0;
        } else {
            // accelerate up to v_max and move the remaining distance at const speed
            t_accl = (v_max - v_res) * a_opt.abs();
            t_v = (delta_position - a_opt * t_accl.powi(2)).abs() / v_max;
        }
        (a_opt, t_accl, t_v)
    }

    /// Set position mode - perform movement path planning
    ///
    /// Move a certain (signed) distance (relative position) and stop.
    ///
    /// Preconditions to be met are:
    ///
    /// 0. start over later if speed or acceleration different from  zero
    ///
    /// 7 steps need to be performed:
    ///
    /// 1. Increase acceleration from zero to an optimum value a_opt max jerk
    /// 2. Keep accelerating at the optimum acceleration for a fixed time t_accl
    /// 3. Reduce acceleration to a zero value
    /// 4. Move at constant speed for a fixed time t_v
    /// 5. Increase breaking acceleration from zero to an optimum value a_opt
    /// 6. Break at constant acceleration for a fixed time t_accl
    /// 7. Reduce breaking acceleration to zero value at max jerk
    ///
    /// This function computes optimum acceleration and time to keep a
    /// constant acceleration as well as optimum speed and time to keep the speed.
    /// It updates state structure representing the
    /// new movement path plan/steps.
    ///
    /// # Arguments
    ///
    /// * `delta_position`: The target position equals current position + (signed) delta position.
    pub fn set_target_relative_position(&mut self, delta_position: f32) {
        if self.current.acceleration != 0.0 || self.current.speed != 0.0 {
            // planning conditions are not given -> go to zero acceleration, zero speed and start over
            // remember current position for later reconsideration
            self.reset_path_states();
            self.state[0] = RampState::RecomputeToSpeed(0.0);
            self.state[1] = RampState::RecomputeToPosition(self.current.position, delta_position);
        } else {
            let (a_opt, t_accl, t_v) = self.plan_for_position(delta_position);

            // Compose the state changes according to a_opt, t_accl, t_v
            self.state[0] = RampState::AccelerateTo(a_opt);
            self.state[1] = RampState::AccelerateConst(t_accl);
            self.state[2] = RampState::AccelerateTo(0.0);
            self.state[3] = RampState::SpeedConst(t_v);
            self.state[4] = RampState::AccelerateTo(-1.0 * a_opt);
            self.state[5] = RampState::AccelerateConst(t_accl);
            self.state[6] = RampState::AccelerateTo(0.0);
            self.state[7] = RampState::SpeedZero;
        }
    }

    // Plan optimal acceleration (a_optimal) and time to accelerate (t_accl) to reach target speed
    // Returns (a_optimal, t_accl) tupel
    fn plan_for_speed(&self, speed: f32) -> (f32, f32) {
        let speed = self.consider_max_speed(speed) - self.current.speed;
        let jerk = self.consider_max_jerk(MAX_JERK);
        let a_optimum = (speed * jerk).abs().sqrt(); // a is with out sign so far
        let a_max = self.consider_max_acceleration(a_optimum * MAX_ACCELERATION);
        if a_optimum < a_max.abs() {
            // no need to accelerate at constant acceleration
            (a_optimum * speed.signum(), 0.0)
        } else {
            // need to run for a certain time at constant acceleration
            (
                a_max * speed.signum(),
                (speed.abs() - a_max.powi(2) / jerk) / a_max,
            )
        }
    }
    /// Set speed mode - perform movement path planning
    ///
    ///
    /// Preconditions to be met are:
    ///
    /// 0. current acceleration equals zero i.e. speed is constant
    ///
    /// 3 steps need to be performed:
    ///
    /// 1. Increase acceleration from zero to a optimum value
    /// 2. Keep accelerating at the optimum acceleration for a fixed time
    /// 3. Reduce acceleration to a zero value
    ///
    /// This function computes optimum acceleration and time to keep a
    /// constant acceleration and update state structure representing the
    /// new movement path plan.
    ///
    /// # Arguments
    ///
    /// * `speed`: The target speed
    ///    Note: if abs(speed) >  max_speed, target speed gets reduced.
    pub fn set_target_speed(&mut self, speed: f32) {
        if self.current.acceleration != 0.0 {
            // planning conditions are not given -> go to zero acceleration and start over
            // keep the remaining step queue intact
            // push states in reverse order
            if self.state[0] == RampState::SpeedZero {
                self.push_state_on_as_current(RampState::SpeedKeep);
            }
            self.push_state_on_as_current(RampState::RecomputeToSpeed(speed));
            self.push_state_on_as_current(RampState::AccelerateTo(0.0));
        } else {
            if speed == self.current.speed {
                if self.state[0] == RampState::SpeedZero {
                    // in case a RecomputeXX is following
                    // do not have the intermediate step of keeping the speed
                    self.push_state_on_as_current(RampState::SpeedKeep);
                }
            } else {
                let (a_optimum, t_accl) = self.plan_for_speed(speed);

                // set the path steps (states) accordingly
                if self.state[0] == RampState::SpeedZero {
                    // in case a RecomputeXX is following
                    // do not have the intermediate step of keeping the speed
                    self.push_state_on_as_current(RampState::SpeedKeep);
                }
                // push states in reverse order
                self.push_state_on_as_current(RampState::AccelerateTo(0.0));
                self.push_state_on_as_current(RampState::AccelerateConst(t_accl));
                self.push_state_on_as_current(RampState::AccelerateTo(a_optimum));
            }
        }
    }

    #[allow(dead_code)]
    /// Determine at wich generalized position the Ramp is at the moment
    pub fn set(&mut self, current: GeneralizedPosition) {
        self.current = current
    }

    /// Determine at wich generalized position the Ramp is at the moment
    pub fn get(&self) -> GeneralizedPosition {
        self.current
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

    /// Update state removes current state (path element) and let the next state
    /// be the new current one
    fn update_state(&mut self) {
        for i in 0..(self.state.len() - 1) {
            self.state[i] = self.state[i + 1];
        }
    }

    /// Push all states one level back and add a new current state
    fn push_state_on_as_current(&mut self, state: RampState) {
        for i in 1..(self.state.len()) {
            self.state[self.state.len() - i] = self.state[self.state.len() - 1 - i];
        }
        self.state[0] = state;
    }

    pub fn move_at_constant_speed(&mut self) -> Option<GeneralizedPosition> {
        // Update the generalized position
        self.current.position += self.current.speed * self.sample_interval;

        // return the new position
        Some(self.current)
    }

    pub fn move_at_speed_for_time(&mut self, time: f32) -> Option<GeneralizedPosition> {
        // Update the position
        self.current.position += self.current.speed * self.sample_interval;

        // Check if the state needs to be updated and update
        let new_time = time - self.sample_interval;
        if new_time > 0.0 {
            self.state[0] = RampState::SpeedConst(new_time);
        } else {
            self.update_state();
        }

        // return the new position
        Some(self.current)
    }

    pub fn accelerate_for_time(&mut self, time: f32) -> Option<GeneralizedPosition> {
        // Update the speed and position
        self.current.speed += self.current.acceleration * self.sample_interval;
        self.current.position += self.current.speed * self.sample_interval;

        // Check if the state needs to be updated and update
        let new_time = time - self.sample_interval;
        if new_time > 0.0 {
            self.state[0] = RampState::AccelerateConst(new_time);
        } else {
            self.update_state();
        }

        // return the new position
        Some(self.current)
    }

    pub fn accelerate_to(&mut self, acceleration: f32) -> Option<GeneralizedPosition> {
        // Update the acceleration, speed and position
        let jerk = (acceleration - self.current.acceleration) / self.sample_interval;
        let jerk = self.consider_max_jerk(jerk);
        self.current.acceleration += jerk * self.sample_interval;
        self.current.speed += self.current.acceleration * self.sample_interval;
        self.current.position += self.current.speed * self.sample_interval;

        // Check if the state needs to be updated and update
        if self.current.acceleration == acceleration {
            self.update_state();
        }

        // return the new position
        Some(self.current)
    }
}

impl Iterator for Ramp {
    type Item = GeneralizedPosition;
    /// Iterate delivers generalized positions towards
    /// the target set (move a distance or move at speed x)
    ///
    fn next(&mut self) -> Option<Self::Item> {
        match self.state[0] {
            RampState::SpeedZero => None,
            RampState::SpeedKeep => self.move_at_constant_speed(),
            RampState::SpeedConst(time) => self.move_at_speed_for_time(time),
            RampState::AccelerateTo(acceleration) => self.accelerate_to(acceleration),
            RampState::AccelerateConst(time) => self.accelerate_for_time(time),
            RampState::RecomputeToSpeed(speed) => {
                self.set_target_speed(speed);
                self.update_state();
                Some(self.current)
            }
            RampState::RecomputeToPosition(original_position, target_position) => {
                self.set_target_relative_position(
                    target_position - self.current.position - original_position,
                );
                self.update_state();
                Some(self.current)
            }
        }
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

    #[should_panic]
    #[test]
    fn ramp_constraints_panic_negative_speed() {
        let _rp = RampConstraints::default().max_speed(-2.0);
    }

    #[test]
    fn ramp_constraints_set_speed() {
        let rp = RampConstraints::default().max_speed(1.0);
        assert_eq!(rp.max_acceleration, None);
        assert_eq!(rp.max_jerk, None);
        assert_eq!(rp.max_speed, Some(1.0));
    }

    #[should_panic]
    #[test]
    fn ramp_constraints_panic_negative_acceleration() {
        let _rp = RampConstraints::default().max_acceleration(-2.0);
    }

    #[test]
    fn ramp_constraints_set_acceleration() {
        let rp = RampConstraints::default().max_acceleration(1.0);
        assert_eq!(rp.max_speed, None);
        assert_eq!(rp.max_acceleration, Some(1.0));
        assert_eq!(rp.max_jerk, None);
    }

    #[should_panic]
    #[test]
    fn ramp_constraints_panic_negative_jerk() {
        let _rp = RampConstraints::default().max_jerk(-2.0);
    }

    #[test]
    fn ramp_constraints_set_jerk() {
        let rp = RampConstraints::default().max_jerk(1.0);
        assert_eq!(rp.max_speed, None);
        assert_eq!(rp.max_acceleration, None);
        assert_eq!(rp.max_jerk, Some(1.0));
    }

    #[test]
    fn consider_max_jerk_ok() {
        let rc = RampConstraints::default().max_jerk(1.0);
        let r = Ramp::new(0.1, rc);
        assert_eq!(r.consider_max_jerk(20.0), 1.0);

        let rc = RampConstraints::default();
        let r = Ramp::new(0.1, rc);
        assert_eq!(r.consider_max_jerk(20.0), 20.0);
    }

    #[test]
    fn consider_max_acceleration_ok() {
        let rc = RampConstraints::default().max_acceleration(1.0);
        let r = Ramp::new(0.1, rc);
        assert_eq!(r.consider_max_acceleration(20.0), 1.0);

        let rc = RampConstraints::default();
        let r = Ramp::new(0.1, rc);
        assert_eq!(r.consider_max_acceleration(20.0), 20.0);
    }

    #[test]
    fn consider_max_speed_ok() {
        let rc = RampConstraints::default().max_speed(1.0);
        let r = Ramp::new(0.1, rc);
        assert_eq!(r.consider_max_speed(20.0), 1.0);

        let rc = RampConstraints::default();
        let r = Ramp::new(0.1, rc);
        assert_eq!(r.consider_max_speed(20.0), 20.0);
    }

    #[test]
    fn update_position_ok() {
        let rp: RampConstraints = Default::default();
        let mut r = Ramp::new(0.1, rp);
        r.state = [
            RampState::SpeedConst(0.0),
            RampState::SpeedConst(1.0),
            RampState::SpeedConst(2.0),
            RampState::SpeedConst(3.0),
            RampState::SpeedConst(4.0),
            RampState::SpeedConst(5.0),
            RampState::SpeedConst(6.0),
            RampState::SpeedConst(7.0),
        ];
        r.update_state();
        assert_eq!(r.state[0], RampState::SpeedConst(1.0));
        assert_eq!(r.state[1], RampState::SpeedConst(2.0));
        assert_eq!(r.state[2], RampState::SpeedConst(3.0));
        assert_eq!(r.state[3], RampState::SpeedConst(4.0));
        assert_eq!(r.state[4], RampState::SpeedConst(5.0));
        assert_eq!(r.state[5], RampState::SpeedConst(6.0));
        assert_eq!(r.state[6], RampState::SpeedConst(7.0));
        assert_eq!(r.state[7], RampState::SpeedConst(7.0));
    }

    #[test]
    fn check_default_position_ok() {
        let rp: RampConstraints = Default::default();
        let r = Ramp::new(0.1, rp);
        assert_general_position_eq!(Some(r.current), s_gp!(0.0_f32, 0.0_f32, 0.0_f32));
    }

    #[test]
    fn move_at_constant_speed_ok() {
        let rc = RampConstraints::default();
        let sample_interval = 0.1; // seconds
        let mut r = Ramp::new(sample_interval, rc);
        // Inject position
        r.current = GeneralizedPosition {
            position: 2.0_f32,
            speed: -1.0_f32,
            acceleration: 0.0_f32,
        };
        assert_general_position_eq!(
            r.move_at_constant_speed(),
            s_gp!(1.9_f32, -1.0_f32, 0.0_f32)
        );
        assert_general_position_eq!(
            r.move_at_constant_speed(),
            s_gp!(1.8_f32, -1.0_f32, 0.0_f32)
        );
    }

    #[test]
    fn move_at_speed_for_time_ok() {
        let rc = RampConstraints::default();
        let sample_interval = 0.1; // seconds
        let mut r = Ramp::new(sample_interval, rc);
        // Inject position
        r.current = GeneralizedPosition {
            position: 2.0_f32,
            speed: -1.0_f32,
            acceleration: 0.0_f32,
        };
        assert_general_position_eq!(
            r.move_at_speed_for_time(2.0),
            s_gp!(1.9_f32, -1.0_f32, 0.0_f32)
        );
        assert_eq!(r.state[0], RampState::SpeedConst(1.9));
        r.state[1] = RampState::SpeedKeep;
        assert_general_position_eq!(
            r.move_at_speed_for_time(0.1),
            s_gp!(1.8_f32, -1.0_f32, 0.0_f32)
        );
        assert_eq!(r.state[0], RampState::SpeedKeep);
    }

    #[test]
    fn accelerate_for_time_ok() {
        let rc = RampConstraints::default();
        let sample_interval = 0.1; // seconds
        let mut r = Ramp::new(sample_interval, rc);
        // Inject position
        r.current = GeneralizedPosition {
            position: 0.0_f32,
            speed: 0.0_f32,
            acceleration: 10.0_f32,
        };
        assert_general_position_eq!(
            r.accelerate_for_time(20.0),
            s_gp!(0.1_f32, 1.0_f32, 10.0_f32)
        );
        assert_eq!(r.state[0], RampState::AccelerateConst(19.9));
        r.state[1] = RampState::SpeedKeep;
        assert_general_position_eq!(
            r.accelerate_for_time(0.1),
            s_gp!(0.3_f32, 2.0_f32, 10.0_f32)
        );
        assert_eq!(r.state[0], RampState::SpeedKeep);
    }

    #[test]
    fn accelerate_to_at_1sec_sample_interval() {
        let rc = RampConstraints::default().max_jerk(1.0);
        let sample_interval = 1.0; // seconds
        let mut r = Ramp::new(sample_interval, rc);
        // Inject position
        r.current = GeneralizedPosition {
            position: 0.0_f32,
            speed: 0.0_f32,
            acceleration: 0.0_f32,
        };
        r.state[0] = RampState::AccelerateTo(10.0);
        r.state[1] = RampState::SpeedKeep;
        assert_general_position_eq!(r.accelerate_to(10.0), s_gp!(1.0_f32, 1.0_f32, 1.0_f32));
        assert_eq!(r.state[0], RampState::AccelerateTo(10.0));
        assert_general_position_eq!(r.accelerate_to(2.0), s_gp!(4.0_f32, 3.0_f32, 2.0_f32));
        assert_eq!(r.state[0], RampState::SpeedKeep);
    }

    macro_rules! assert_gp_accl_eq {
        ($left:expr, $right:expr) => {
            match $left {
                Some(gp) => {
                    if ! approx_eq!(f32, gp.acceleration, $right, epsilon = 0.001) {
                        panic!("assertion failed, Acceleration does not match,  left:  {:?}\n  right: {:?}", $left, $right)
                    }
                },
                None =>  panic!("assertion failed,  left:  {:?}\n  right: {:?}", $left, $right)
            }
        }
    }

    #[test]
    fn accelerate_to_at_1_tenth_of_a_sec_sample_interval() {
        let rc = RampConstraints::default().max_jerk(1.0);
        let sample_interval = 0.1; // seconds
        let mut r = Ramp::new(sample_interval, rc);
        // Inject position
        r.current = GeneralizedPosition {
            position: 0.0_f32,
            speed: 0.0_f32,
            acceleration: 0.0_f32,
        };
        r.state[0] = RampState::AccelerateTo(1.0);
        r.state[1] = RampState::SpeedKeep;
        assert_general_position_eq!(r.accelerate_to(1.0), s_gp!(0.001_f32, 0.01_f32, 0.1_f32));
        assert_general_position_eq!(r.accelerate_to(1.0), s_gp!(0.004_f32, 0.03_f32, 0.2_f32));
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.3_f32);
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.4_f32);
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.5_f32);
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.6_f32);
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.7_f32);
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.8_f32);
        assert_gp_accl_eq!(r.accelerate_to(1.0), 0.9_f32);
        assert_eq!(r.state[0], RampState::AccelerateTo(1.0));
        assert_gp_accl_eq!(r.accelerate_to(1.0), 1.0_f32);
        assert_eq!(r.state[0], RampState::SpeedKeep);
    }

    macro_rules! assert_float_2tuple_eq {
        ($a: expr, $b:expr, $epsilon: expr) => {
            if !approx_eq!(f32, $a.0, $b.0, epsilon = $epsilon)
                || !approx_eq!(f32, $a.1, $b.1, epsilon = $epsilon)
            {
                panic!("assertion failed\n  left:  {:?}\n  right: {:?}", $a, $b);
            }
        };
    }

    macro_rules! assert_float_3tuple_eq {
        ($a: expr, $b:expr, $epsilon: expr) => {
            if !approx_eq!(f32, $a.0, $b.0, epsilon = $epsilon)
                || !approx_eq!(f32, $a.1, $b.1, epsilon = $epsilon)
                || !approx_eq!(f32, $a.2, $b.2, epsilon = $epsilon)
            {
                panic!("assertion failed\n  left:  {:?}\n  right: {:?}", $a, $b);
            }
        };
    }

    #[test]
    fn try_test() {
        let mut a = -22894.0_f32;
        assert_eq!(a.signum(), -1.0);
        assert_eq!((a * -1.0).signum(), 1.0);
    }

    #[test]
    fn compute_parameters_of_acceleration_change_steps_without_constraints() {
        let rc = RampConstraints::default();
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(1.0),
            (9085.604, 0.0),
            0.001
        );
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(-1.0),
            (-9085.604, 0.0),
            0.001
        );
    }

    #[test]
    fn compute_parameters_of_acceleration_change_steps_with_speed_constraints() {
        let rc = RampConstraints::default().max_speed(0.01);
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(10.0),
            (100.0, 10.0),
            0.001
        );
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(-10.0),
            (-100.0, -10.0),
            0.001
        );
    }

    #[test]
    fn compute_parameters_of_acceleration_change_steps_with_acceleration_constraints() {
        let rc = RampConstraints::default().max_acceleration(1.0);
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(1.0),
            (1.0, 1.0),
            0.001
        );
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(-1.0),
            (-1.0, -1.0),
            0.001
        );
    }

    #[test]
    fn compute_parameters_of_acceleration_change_steps_with_jerk_constraints() {
        let rc = RampConstraints::default().max_jerk(1.0);
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(1.0),
            (0.908, 0.0),
            0.001
        );
        assert_float_2tuple_eq!(
            r.compute_parameters_of_acceleration_change_steps(-1.0),
            (-0.908, 0.0),
            0.001
        );
    }

    #[test]
    fn plan_for_position_without_constraints() {
        let rc = RampConstraints::default();
        let r = Ramp::new(1.0, rc);
        assert_float_3tuple_eq!(r.plan_for_position(1.0), (9085.604, 0.0, 0.0), 0.001);
        assert_float_3tuple_eq!(r.plan_for_position(-1.0), (-9085.604, 0.0, 0.0), 0.001);
    }

    #[test]
    fn plan_for_position_with_speed_constraints() {
        let rc = RampConstraints::default().max_speed(0.01);
        let r = Ramp::new(1.0, rc);
        assert_float_3tuple_eq!(r.plan_for_position(10.0), (100.0, 0.0, 1000.0), 0.001);
        assert_float_3tuple_eq!(r.plan_for_position(-10.0), (-100.0, 0.0, 1000.0), 0.001);
    }

    #[test]
    fn plan_for_position_with_acceleration_constraints() {
        let rc = RampConstraints::default().max_acceleration(1.0);
        let r = Ramp::new(1.0, rc);
        assert_float_3tuple_eq!(r.plan_for_position(10.0), (1.0, 3.162, 0.0), 0.001);
        assert_float_3tuple_eq!(r.plan_for_position(-10.0), (-1.0, 3.162, 0.0), 0.001);
    }

    #[test]
    fn plan_for_position_with_jerk_constraints() {
        let rc = RampConstraints::default().max_jerk(0.1);
        let r = Ramp::new(1.0, rc);
        assert_float_3tuple_eq!(r.plan_for_position(1.0), (0.195, 0.0, 0.0), 0.001);
        assert_float_3tuple_eq!(r.plan_for_position(-1.0), (-0.195, 0.0, 0.0), 0.001);
    }

    #[test]
    fn plan_for_position_with_all_constraints() {
        let rc = RampConstraints::default()
            .max_jerk(1.0)
            .max_acceleration(1.0)
            .max_speed(10.0);
        let r = Ramp::new(1.0, rc);
        assert_float_3tuple_eq!(r.plan_for_position(100.0), (1.0, 9.0, 1.766), 0.001);
        assert_float_3tuple_eq!(r.plan_for_position(-100.0), (-1.0, 9.0, 1.766), 0.001);
    }

    #[test]
    fn plan_for_speed_without_constraints() {
        let rc = RampConstraints::default();
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(r.plan_for_speed(10.0), (3162.2, 0.0), 0.1);
    }

    #[test]
    fn plan_for_speed_with_speed_constraints() {
        let rc = RampConstraints::default().max_speed(1.0);
        let mut r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(r.plan_for_speed(10.0), (1000.0, 0.0), 0.2);
        assert_float_2tuple_eq!(r.plan_for_speed(-10.0), (-1000.0, 0.0), 0.2);
        r.current.speed = 2.0;
        assert_float_2tuple_eq!(r.plan_for_speed(10.0), (-1000.0, 0.0), 0.2);
    }

    #[test]
    fn plan_for_speed_with_acceleration_constraints() {
        let rc = RampConstraints::default().max_acceleration(1.0);
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(r.plan_for_speed(10.0), (1.0, 10.0), 0.1);
        assert_float_2tuple_eq!(r.plan_for_speed(-10.0), (-1.0, 10.0), 0.1);
    }

    #[test]
    fn plan_for_speed_with_jerk_constraints() {
        let rc = RampConstraints::default().max_jerk(1.0);
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(r.plan_for_speed(10.0), (3.162, 0.0), 0.1);
        assert_float_2tuple_eq!(r.plan_for_speed(-10.0), (-3.162, 0.0), 0.1);
    }

    #[test]
    fn plan_for_speed_with_all_constraints() {
        let rc = RampConstraints::default()
            .max_jerk(1.0)
            .max_acceleration(1.0)
            .max_speed(100.0);
        let r = Ramp::new(1.0, rc);
        assert_float_2tuple_eq!(r.plan_for_speed(10.0), (1.0, 9.0), 0.1);
    }

    #[test]
    fn set_target_speed_at_clean_conditions_wo_followup() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);

        r.set_target_speed(1.0);
        // assert_eq!(r.state[0], RampState::AccelerateTo(x));
        assert_eq!(r.state[1], RampState::AccelerateConst(0.0));
        assert_eq!(r.state[2], RampState::AccelerateTo(0.0));
        assert_eq!(r.state[3], RampState::SpeedKeep);
    }

    #[test]
    fn set_target_speed_at_clean_conditions_with_followup() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);

        r.state[0] = RampState::RecomputeToPosition(1.0, 1.0);
        r.set_target_speed(1.0);
        assert_eq!(r.state[1], RampState::AccelerateConst(0.0));
        assert_eq!(r.state[2], RampState::AccelerateTo(0.0));
        assert_eq!(r.state[3], RampState::RecomputeToPosition(1.0, 1.0));
    }

    #[test]
    fn set_target_speed_at_acceleration_conditions_wo_followup() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);
        r.current.acceleration = 1.0;

        r.set_target_speed(1.0);
        assert_eq!(r.state[0], RampState::AccelerateTo(0.0));
        assert_eq!(r.state[1], RampState::RecomputeToSpeed(1.0));
        assert_eq!(r.state[2], RampState::SpeedKeep);
    }

    #[test]
    fn set_target_speed_at_acceleration_conditions_with_followup() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);
        r.current.acceleration = 1.0;
        r.state[0] = RampState::RecomputeToPosition(1.0, 1.0);

        r.set_target_speed(1.0);
        assert_eq!(r.state[0], RampState::AccelerateTo(0.0));
        assert_eq!(r.state[1], RampState::RecomputeToSpeed(1.0));
        assert_eq!(r.state[2], RampState::RecomputeToPosition(1.0, 1.0));
    }

    #[test]
    fn set_target_relative_position_at_clean_condition() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);

        r.set_target_relative_position(1.0);
        assert_eq!(r.state[2], RampState::AccelerateTo(0.0));
        assert_eq!(r.state[6], RampState::AccelerateTo(0.0));
    }

    #[test]
    fn push_state_as_current_ok() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);

        r.push_state_on_as_current(RampState::SpeedKeep);
        assert_eq!(r.state[0], RampState::SpeedKeep);
        assert_eq!(r.state[1], RampState::SpeedZero);

        r.push_state_on_as_current(RampState::AccelerateTo(1.0));
        assert_eq!(r.state[0], RampState::AccelerateTo(1.0));
        assert_eq!(r.state[1], RampState::SpeedKeep);
        assert_eq!(r.state[2], RampState::SpeedZero);

        r.push_state_on_as_current(RampState::SpeedConst(1.0));
        assert_eq!(r.state[0], RampState::SpeedConst(1.0));
        assert_eq!(r.state[1], RampState::AccelerateTo(1.0));
        assert_eq!(r.state[2], RampState::SpeedKeep);
        assert_eq!(r.state[3], RampState::SpeedZero);
    }

    #[test]
    fn set_target_relative_position_recompute_conditions() {
        let rc = RampConstraints::default();
        let mut r = Ramp::new(1.0, rc);

        r.current.acceleration = 1.0;
        r.set_target_relative_position(1.0);
        assert_eq!(r.state[0], RampState::RecomputeToSpeed(0.0));
        assert_eq!(r.state[1], RampState::RecomputeToPosition(0.0, 1.0));
        assert_eq!(r.state[2], RampState::SpeedZero);

        r.current.acceleration = 0.0;
        r.current.speed = 1.0;
        r.set_target_relative_position(1.0);
        assert_eq!(r.state[0], RampState::RecomputeToSpeed(0.0));
        assert_eq!(r.state[1], RampState::RecomputeToPosition(0.0, 1.0));
        assert_eq!(r.state[2], RampState::SpeedZero);

        r.current.acceleration = 1.0;
        r.current.speed = 1.0;
        r.set_target_relative_position(1.0);
        assert_eq!(r.state[0], RampState::RecomputeToSpeed(0.0));
        assert_eq!(r.state[1], RampState::RecomputeToPosition(0.0, 1.0));
        assert_eq!(r.state[2], RampState::SpeedZero);
    }
}
