// Use a pid target setter as a mutex in different threads
// This is necessary since the Iterator trait requires
// &mut self as parameter in the next function
//
// main function spawns a thread the consumes the iterator
// every 500 milli seconds.
// in the main thread after the first second the target value is set to 1
// and after another second the target is set to 2.
// Expected output shows the changing value received through the
// iterator.

use std::sync::{Arc, Mutex};
use std::thread;
use std::thread::sleep;
use std::time::Duration;

use conreg::target::*;

const HALF_A_SEC: Duration = Duration::from_millis(500);
const ONE_SEC: Duration = Duration::from_millis(1000);

fn main() {
    let target = Arc::new(Mutex::new(TargetValue(0_isize)));

    let target_clone = Arc::clone(&target);
    let handle = thread::spawn(move || {
        for _v in 1..7 {
            {
                let mut t = target_clone.lock().unwrap();
                println!("Value {:?}", t.next());
            }
            sleep(HALF_A_SEC);
        }
    });
    sleep(ONE_SEC);
    {
        let mut t = target.lock().unwrap();
        t.set(1_isize);
    }
    sleep(ONE_SEC);
    {
        let mut t = target.lock().unwrap();
        t.set(2_isize);
    }
    handle.join().unwrap();
}
