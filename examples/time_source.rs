//! Print values of a random sensor
//!

use sensact::timebase;
use sensact::TimeSourceIterator;

fn main() {
    let t = timebase::StandardTimeSource;
    match t.next {
        Some(v) => println!("Sensor random: {}", v);
        None => panic("Should never happen");
    }
    let  = t.next();

}
