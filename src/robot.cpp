#include "api.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"

// 2496v
#define LF_PORT 19
#define LB_PORT 11
#define RF_PORT 10
#define RB_PORT 1
// all ABOVE MOTORS NEED TO BE CHANGED FOR 2496S CHASSIS ----these things arent reversed make sure they actually run----
#define IMU_PORT 20
#define intake_PORT 15
#define lift_PORT1 7
#define lift_PORT2 9
#define Distance_PORTL 3 //none for now
#define Distance_PORTR 2
// Front/back distance sensor ports (set to unused ports by default)


// rotation sensors
pros::Distance distanceSensorL(Distance_PORTL);
pros::Distance distanceSensorR(Distance_PORTR);
// optional front/back sensors (default to same ports as left/right until hardware configured)

//okay man
// chassis motors
pros::Motor LF(LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB(LB_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RF(RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB(RB_PORT, pros::E_MOTOR_GEARSET_06, true);

// intake motors
pros::Motor_Group lift({lift_PORT1, lift_PORT2});
pros::Motor_Group& Lift = lift;
pros::Motor intake(intake_PORT, pros::E_MOTOR_GEARSET_06, false);

void configure_lift_brake_mode() {
    pros::c::motor_set_brake_mode(lift_PORT1, pros::E_MOTOR_BRAKE_HOLD);
    pros::c::motor_set_brake_mode(lift_PORT2, pros::E_MOTOR_BRAKE_HOLD);
}

// pneumatics
pros::ADIDigitalOut scraper('A');
pros::ADIDigitalOut& claw = scraper;
pros::ADIDigitalOut clawTilt('C');
pros::ADIDigitalOut Iain_kim('B');

// sensors
pros::Imu imu(IMU_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);
