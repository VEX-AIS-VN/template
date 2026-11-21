use std::time::{Duration, Instant};

use evian::{control::loops::{Feedback, Pid}, drivetrain::{Drivetrain, model::{Differential, Tank}}, tracking::{TracksHeading, TracksPosition}};
use shrewnit::Meters;
use vexide::{smart::motor::Motor, time::sleep};

use crate::units::Translation;

pub struct MoveToPoint {
    pub linear_pid: Pid,
    pub angular_pid: Pid,
    pub tolerance: f64
}

pub async fn move_to_point(
    drivetrain: &mut Drivetrain<Differential, impl TracksPosition + TracksHeading>,
    constants: &mut MoveToPoint,
    point: Translation,
    max_voltage: f64
) {
    let mut prev = Instant::now();

    loop {

        let dt = Instant::now() - prev;
        let delta: Translation = &point - &Translation::from(drivetrain.tracking.position());
        let dist = delta.norm().to::<Meters>();

        if dist < constants.tolerance {
            drivetrain.model.drive_tank(0.0, 0.0);
            break;
        }

        let linpid = constants.linear_pid.update(0.0, dist, dt);

        let target_angle = delta.y.to::<Meters>().atan2(delta.x.to::<Meters>());
        let anpid = constants.angular_pid.update(drivetrain.tracking.heading().as_radians(), target_angle, dt);

        let left_voltage = linpid - anpid;
        let right_voltage = linpid + anpid;
        
        let scale = left_voltage.abs().max(right_voltage.abs()) / max_voltage;

        if scale > 1.0 {
            drivetrain.model.drive_tank(left_voltage / scale, right_voltage / scale);
        }
        else {
            drivetrain.model.drive_tank(left_voltage, right_voltage);
        }

        println!("{} {}", drivetrain.tracking.position().x(), drivetrain.tracking.position().y());

        prev = Instant::now();

        sleep(Motor::WRITE_INTERVAL).await;
    }
}
