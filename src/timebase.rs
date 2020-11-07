use std::time::Instant;

use crate::TimeSourceIterator;

pub struct StandardTimeSource;

impl TimeSourceIterator for StandardTimeSource {
    type MicroSeconds = u128;

    fn next(&mut self) -> Option<Self::MicroSeconds> {
        let start = Instant::now();
        Some(start.elapsed().as_micros())
    }
}
