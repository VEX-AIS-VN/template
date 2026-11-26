use std::time::{Duration, Instant};
use core::f64::consts::{TAU, PI};

use evian::{control::loops::{Feedback, Pid}, drivetrain::{Drivetrain, model::{Differential, Tank}}, tracking::{TracksHeading, TracksPosition}};
use shrewnit::{Meters, Radians};
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
        prev = Instant::now();
        let position = drivetrain.tracking.position();
        let delta: Translation = &point - &Translation::from(position);
        let dist = delta.norm().to::<Meters>();

        if dist < constants.tolerance {
            let _ = drivetrain.model.drive_tank(0.0, 0.0);
            break;
        }

        let linpid = constants.linear_pid.update(0.0, dist, dt);

        let mut target_angle = delta.heading().to::<Radians>();
        if target_angle > PI {
            target_angle = TAU - target_angle;
        }
        let anpid = constants.angular_pid.update(drivetrain.tracking.heading().as_radians(), target_angle, dt);

        let left_voltage = linpid - anpid;
        let right_voltage = linpid + anpid;
        
        let scale = left_voltage.abs().max(right_voltage.abs()) / max_voltage;

        if scale > 1.0 {
            let _ = drivetrain.model.drive_tank(left_voltage / scale, right_voltage / scale);
        }
        else {
            let _ = drivetrain.model.drive_tank(left_voltage, right_voltage);
        }

        sleep(Motor::WRITE_INTERVAL).await;
    }
}
