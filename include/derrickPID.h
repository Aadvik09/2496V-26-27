#ifndef DERRICKPIDH
#define DERRICKPIDH

#include "api.h"
#include "main.h"
#include "robot.h"

// UNIVERSAL HEADING
extern double universal_target_heading;

// Drive PID constants
extern double driveKP;
extern double driveKI;
extern double driveKD;
extern double driveMAXI;

// Heading correction constants
extern double HCKP;
extern double HCKI;
extern double HCKD;
extern double HCMAXI;

// Wall PID constants
extern double wallKP;
extern double wallKI;
extern double wallKD;

// Turn PID constants
extern double turnKP;
extern double turnKI;
extern double turnKD;
extern double turnMAXI;

extern bool liftTaskRunning;

// Utility functions
extern void chasMove(int left, int right);
extern void resetEncoders();
extern void chasBrake();
extern void chasSlow(int speed, int ms);
extern void LiftScore();

// PID calc
extern double calcPID(int error, double kP, double kI, double kD, double totalError, double prevError, double integralThreshold, double maxI);

// Drive functions
extern void drivePID(int desiredValue, int maxSpeed, int timeout, int chainValue = 0, int dec_point = -1, int errorThreshold = 15, int settleCount = 50, int minSpeed = 30, int triggerDist = -1, int triggerSpeed = 0);
extern void drivePID_distance(int desiredValue, int maxSpeed, int timeout, int wallDistanceTarget, int sensorSide, int dec_point, int chainValue = 0, int errorThreshold = 15, int settleCount = 50, int minSpeed = 30, int triggerDist = -1, int triggerSpeed = 0);
extern void drivePID_distancefront(int desiredValue, int maxSpeed, int timeout, int wallDistanceTarget, int dec_point, int chainValue = 0, int errorThreshold = 15, int settleCount = 50, int minSpeed = 30, int triggerDist = -1, int triggerSpeed = 0);
extern void drivePID_distanceback(int desiredValue, int maxSpeed, int timeout, int wallDistanceTarget, int dec_point, int chainValue = 0, int errorThreshold = 15, int settleCount = 50, int minSpeed = 30, int triggerDist = -1, int triggerSpeed = 0);

// Turn functions
extern void turnPID(double desiredValue, int topSpeed, int timeout, double errorThreshold = 15, int settleCount = 50, double chainValue = 0);

// Arc functions
extern void driveArcL(double theta, double radius, int timeout, int speed, int chainValue = 0, int errorThreshold = 15, int settleCount = 50);
extern void driveArcR(double theta, double radius, int timeout, int speed, int chainValue = 0, int errorThreshold = 15, int settleCount = 50);

// Lift
extern void liftPID(int derivedValue, int maxSpeed, int timeout, int dec_point = -1, int errorThreshold = 100, int settleCount = 10, int minSpeed = 50);

#endif