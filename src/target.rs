//! # Target Value Input
//!
//! Iterators that are intended as target input for controllers
//!
//! ## Constant Target Value
//!
//! ```rust
//! use sensact::target::*;
//!
//! let mut target = TargetValue(0_isize);
//! for _v in 0..2 { println!("Value is {:?}", target.next()); }
//! target.set(1);
//! for _v in 0..2 { println!("Value is {:?}", target.next()); }
//! ```
use core::iter::Iterator;
pub struct TargetValue<T>(pub T);

impl<T: Copy> TargetValue<T> {
    pub fn set(&mut self, value: T) {
        self.0 = value;
    }
}

impl<T: Copy> Iterator for TargetValue<T> {
    type Item = T;
    /// The iterator never finished and returns zero
    /// and never changes the value underneath
    fn next(&mut self) -> Option<Self::Item> {
        Some(self.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn target_value_ok() {
        let mut t = TargetValue(1_isize);
        assert_eq!(t.next(), Some(1_isize));
        t.set(2_isize);
        assert_eq!(t.next(), Some(2_isize));
    }
}
