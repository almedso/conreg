#[cfg(feature = "std")]
use std::time::Instant;

#[cfg(feature = "std")]
fn now_as_micros() -> usize {
    let start = Instant::now();
    let elapsed = start.elapsed().as_micros();
    // do not care if there is a potential overflow
    elapsed as usize
}

const OUTDATED_TIME_DIFFERENCE_IN_MICROSECONDS: usize = 1_000_000; // one second

/// # Delta Time Source
///
/// The time passed between two subsequent calls is returned.
/// This Delta Time Source should be owned by just one consumer. This means
/// it is intended that exactly one consumer wants to know time differences.
/// Designing it like this, does not require the Delta Time Source to
/// know anything about it's consumer.
///
/// ## Examples
///
/// ```rust
/// use steer_and_control::timebase::TimeDelta;
///
/// let mut e1 = TimeDelta::default();
/// let mut e2 = TimeDelta::default();
/// let d = e1.diff();
/// println!("delta 1 in micro seconds {}", d);
/// let d = e2.diff();
/// println!("delta 2 in micro seconds {}", d);
/// let d = e1.diff();
/// println!("delta 1 in micro seconds {}", d);
/// let d = e2.diff();
/// println!("delta 2 in micro seconds {}", d);
/// ```

pub type MicroSeconds = usize;

pub struct TimeDelta {
    now_fn: fn() -> usize,
    last: usize,
}

impl Default for TimeDelta {
    fn default() -> Self {
        Self::new(now_as_micros, 0)
    }
}

impl TimeDelta {
    pub fn new(now_fn: fn() -> usize, last: usize) -> Self {
        Self { now_fn, last }
    }

    /// Computes time difference between this and previous call in microseconds
    ///
    /// It returns returns zero iff more than
    /// OUTDATED_TIME_DIFFERENCE_IN_MICROSECONDS has passed.
    ///
    /// First  time call the difference to object creation time is returned.
    pub fn diff(&mut self) -> MicroSeconds {
        // query the single source for current time in micro seconds
        let now = (self.now_fn)();
        let mut delta: usize;
        if now >= self.last {
            delta = now - self.last;
        } else {
            // handle number overflow
            delta = usize::MAX - self.last;
            delta += now;
            delta += 1;
        }
        self.last = now;
        if delta > OUTDATED_TIME_DIFFERENCE_IN_MICROSECONDS {
            return 0;
        }
        delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn time_delta_ok() {
        assert_eq!(TimeDelta::new(|| 1000_001, 0).diff(), 0);
        assert_eq!(TimeDelta::new(|| 0, 0).diff(), 0);
        assert_eq!(TimeDelta::new(|| 1, 0).diff(), 1);
        assert_eq!(TimeDelta::new(|| 1, usize::MAX - 1).diff(), 3);
    }
}
