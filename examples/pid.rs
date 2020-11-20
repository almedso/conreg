//! Print values of a random sensor
//!
use std::iter;
use steer_and_control::controller::pid::PidController;
use steer_and_control::controller::pid_parameter::{PidParameterAdditive, PidParameterStandard};

fn main() {
    let step_pre = iter::repeat(0.0_f32).take(10);
    let step_post = iter::repeat(1.0_f32).take(100);
    let desired = step_pre.chain(step_post);
    let process = iter::repeat(0.0_f32).take(110);
    let manipulated = desired.zip(process);
    let m2 = manipulated.clone();

    let p = PidParameterAdditive::new(0.5)
        .set_integral(0.1)
        .set_differential(2.0);
    let mut pid = PidController::<f32>::new(0.000_01).set(p);
    println!("PidController: {:?}", pid);

    let manipulated = manipulated.map(|x| pid.control(x));
    let v = manipulated.collect::<Vec<f32>>();
    println!("Result {:?}", v);

    let mut pid = PidController::<i32>::new(0.000_01).set(p);
    println!("PidController: {:?}", pid);

    let v = m2
        .map(|(a, b)| ((a * 1000.0) as i32, (b * 1000.0) as i32))
        .map(|x| pid.control(x))
        .collect::<Vec<i32>>();
    println!("Result {:?}", v);

    let p = PidParameterStandard::new(1.0)
        .set_integral_time(0.5)
        .set_differential_time(0.3);
    println!("PidParameterStandard: {:?}", p);
    let p: PidParameterAdditive = p.into();
    println!("PidParameterAditive: {:?}", p);

    let pid = PidController::<i32>::new(0.0001).set(p);
    println!("PidController: {:?}", pid);
}
