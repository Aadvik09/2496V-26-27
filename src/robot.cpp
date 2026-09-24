#include "api.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"

// 2496v
#define LF_PORT 14
#define LM_PORT 12
#define LB_PORT 15
#define RF_PORT 3
#define RM_PORT 4
#define RB_PORT 19
// all ABOVE MOTORS NEED TO BE CHANGED FOR 2496S CHASSIS ----these things arent reversed make sure they actually run----
#define IMU_PORT 17
#define intake_PORT 11
#define Lift_PORT1 13
#define Lift_PORT2 -2
#define ROTO_PORT 6
#define Distance_PORTL 2
#define Distance_PORTR 11
// Front/back distance sensor ports (set to unused ports by default)
#define Distance_PORTF 4
#define Distance_PORTB 5

// rotation sensors
pros::Rotation roto(ROTO_PORT);
pros::Distance distanceSensorL(Distance_PORTL);
pros::Distance distanceSensorR(Distance_PORTR);
// optional front/back sensors (default to same ports as left/right until hardware configured)
pros::Distance distanceSensorF(Distance_PORTF);
pros::Distance distanceSensorB(Distance_PORTB);
//okay man
// chassis motors
pros::Motor LF(LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM(LM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB(LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF(RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM(RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB(RB_PORT, pros::E_MOTOR_GEARSET_06, false);

// intake motors
pros::Motor_Group Lift({Lift_PORT1, Lift_PORT2});
pros::Motor intake(intake_PORT, pros::E_MOTOR_GEARSET_06, false);

void configure_lift_brake_mode() {
    pros::c::motor_set_brake_mode(Lift_PORT1, pros::E_MOTOR_BRAKE_HOLD);
    pros::c::motor_set_brake_mode(Lift_PORT2, pros::E_MOTOR_BRAKE_HOLD);
}

// pneumatics
pros::ADIDigitalOut claw('A');
pros::ADIDigitalOut clawTilt('C');
pros::ADIDigitalOut Iain_kim('B');

// sensors
pros::Imu imu(IMU_PORT);
pros::Controller con(pros::E_CONTROLLER_MASTER);


