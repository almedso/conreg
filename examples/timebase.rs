use conreg::timebase::*;

fn main() {
    let mut e1 = TimeDelta::default();
    let mut e2 = TimeDelta::default();
    let d = e1.diff();
    println!("delta 1 in micro seconds {}", d);
    let d = e2.diff();
    println!("delta 2 in micro seconds {}", d);
    let d = e1.diff();
    println!("delta 1 in micro seconds {}", d);
    let d = e2.diff();
    println!("delta 2 in micro seconds {}", d);
}
