#ifndef ROBOTH
#define ROBOTH

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

// controller
extern pros::Controller con;

// rotation sensors
extern pros::Rotation roto;

// distance sensors
extern pros::Distance distanceSensorL;
extern pros::Distance distanceSensorR;
extern pros::Distance distanceSensorF;
extern pros::Distance distanceSensorB;

// chassis motors
extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB;


// claw/mechanism motors
extern pros::Motor claw;
extern pros::Motor_Group lift;
extern pros::Motor_Group toggles;
extern pros::Motor_Group& Lift;

void configure_lift_brake_mode();

// imu
extern pros::Imu imu;

// pneumatics
extern pros::ADIDigitalOut clawTilt;
extern pros::ADIDigitalOut Iain_kim;
 

// constants
extern const unsigned int PutPct[];
extern const unsigned int PutVol[];

// timer struct
struct Timer {
    unsigned long start = 0;
    void reset() { start = pros::millis(); }
    unsigned long elapsed() { return pros::millis() - start; }
};



#endif
