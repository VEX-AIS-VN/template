use core::time::Duration;

use shrewnit::{Centimeters, Inches, Meters};
use vexide::prelude::*;
use evian::{
    control::loops::{AngularPid, Pid}, drivetrain::model::Differential, motion::{Basic, Seeking}, prelude::*, tracking::shared_motors
};

mod units;
mod odometry;
mod claw;
mod movement;

use movement::move_to_point;
use odometry::{DifferentialDriveOdometry, DrivetrainConstants};

use crate::{claw::Arm, movement::MoveToPoint, units::Translation};

struct Robot {
    drivetrain: Drivetrain<Differential, DifferentialDriveOdometry>,
    //arm: Arm,
    controller: Controller
}

impl Robot {
    const LINEAR_PID: Pid = Pid::new(2.0, 0.001, 0.0101, Some(3.0));
    const LATERAL_PID: Pid = Pid::new(1.0, 0.001, 0.004, Some(2.0));
    const ANGULAR_PID: AngularPid = AngularPid::new(0.0, 0.0, 0.0, None);
    const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(4.0)
        .velocity(0.25)
        .duration(Duration::from_millis(15));
    const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(f64::to_radians(8.0))
        .velocity(0.09)
        .duration(Duration::from_millis(15));
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        
    }

    async fn driver(&mut self) {
        println!("hi");
        let mut basic = Basic {
            linear_controller: Self::LINEAR_PID,
            angular_controller: Self::ANGULAR_PID,
            linear_tolerances: Self::LINEAR_TOLERANCES,
            angular_tolerances: Self::ANGULAR_TOLERANCES,
            timeout: Some(Duration::from_secs(10)),
        };

        let mut seeking = Seeking {
            linear_controller: Self::LINEAR_PID,
            lateral_controller: Self::LATERAL_PID,
            tolerances: Tolerances::new()
                .error(0.02),
            timeout: None,
        };

        let mut constants = MoveToPoint {
            linear_pid: Self::LINEAR_PID,
            angular_pid: Self::LATERAL_PID,
            tolerance: 0.1
        };

        move_to_point(&mut self.drivetrain, &mut constants, Translation::new(1.0 * Meters, 1.0 * Meters), 0.3).await;

        loop {
            let state = self.controller.state().unwrap_or_default();

            _ = self
                .drivetrain
                .model
                .drive_arcade(state.left_stick.x(), state.left_stick.y());

            sleep(Motor::WRITE_INTERVAL).await;
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let left_motors = shared_motors![
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward)
    ];

    let right_motors = shared_motors![
        Motor::new(peripherals.port_10, Gearset::Green, Direction::Reverse)
    ];

    let constants = DrivetrainConstants {
        wheel_diameter: 4.0 * Inches,
        wheel_from_center: 15.0 * Centimeters
    };

    Robot {
        drivetrain: Drivetrain::new(
            Differential::from_shared(left_motors.clone(), right_motors.clone()),
            DifferentialDriveOdometry::new(constants, None, left_motors, right_motors)
        ),

        /*arm: Arm::new(
            Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
            Motor::new(peripherals.port_13, Gearset::Green, Direction::Forward),
            AngularPid::new(1.0, 2.0, 0.125, None)
        ),*/

        controller: peripherals.primary_controller
    }
    .compete()
    .await;
}
