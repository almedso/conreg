use core::default::Default;
use steer_and_control::steerer::ramp::{Ramp, RampConstraints};

fn main() {
    let rc: RampConstraints = Default::default();
    let rc = rc.max_speed(1.0);
    println!("{:?}", rc);

    let mut rp = Ramp::new(0.1, rc);
    // Position Mode: Stop in a distance of - 10
    rp.set_target_distance(10.0);

    let rs = rp.clone();
    let ra = rp.clone();

    let takes = 10_usize;
    println!("Current generalized position{:?}", rp.get());

    let position: Vec<f32> = rp.take(takes).map(|x| x.position).collect::<Vec<f32>>();
    println!("Positions {:?}", position);

    let speed: Vec<f32> = rs.take(takes).map(|x| x.speed).collect::<Vec<f32>>();
    println!("Positions {:?}", speed);

    let acceleration: Vec<f32> = ra.take(takes).map(|x| x.acceleration).collect::<Vec<f32>>();
    println!("Positions {:?}", acceleration);
}
