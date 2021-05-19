use conreg::control::pid::PidController;
use conreg::control::pid_parameter::PidParameterAdditive;
use std::iter;

fn main() {
    let step_pre = iter::repeat(0.0_f32).take(5);
    let step_post = iter::repeat(1.0_f32).take(25);
    let desired = step_pre.chain(step_post);
    let process = iter::repeat(0.0_f32).take(30);
    let manipulated = desired.zip(process);

    let p = PidParameterAdditive::new(1.0)
        .set_integral(2.0)
        .set_differential(0.5);
    let mut pid = PidController::<f32>::new_with_t1(0.04, 0.1).set(p);
    println!("PidController: {:?}", pid);

    let manipulated = manipulated.map(|x| pid.control(x));
    // let mut count = 100_i32;
    // let manipulated = manipulated.filter(|_x| {
    //     count -= 1;
    //     if count == 0 {
    //         count = 100;
    //         return true;
    //     }
    //     false
    // });
    let v = manipulated.collect::<Vec<f32>>();
    println!("Result {:?}", v);

    let mut pid = PidController::<i32>::new_with_t1(0.04, 0.1).set(p);
    println!("PidController: {:?}", pid);

    let step_pre = iter::repeat(0i32).take(5);
    let step_post = iter::repeat(1000i32).take(25);
    let desired = step_pre.chain(step_post);
    let process = iter::repeat(0i32).take(30);
    let manipulated = desired.zip(process);

    let v = manipulated
        // .map(|(a, b)| ((a * 1000.0) as i32, (b * 1000.0) as i32))
        .map(|x| pid.control(x))
        .collect::<Vec<i32>>();
    println!("Result {:?}", v);

    // let p = PidParameterStandard::new(1.0)
    //     .set_integral_time(0.5)
    //     .set_differential_time(0.3);
    // println!("PidParameterStandard: {:?}", p);
    // let p: PidParameterAdditive = p.into();
    // println!("PidParameterAditive: {:?}", p);

    // let pid = PidController::<i32>::new(0.0001).set(p);
    // println!("PidController: {:?}", pid);
}
