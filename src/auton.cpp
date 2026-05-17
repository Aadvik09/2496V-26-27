#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "derrickPID.h"


using namespace std;
using namespace pros;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or t
 * he VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */


void autonomous() {

  //KEY (LOOK HERE IF YOU ARE LOST ON WHAT PARAMETERS MEAN)
  // each function has a bunch of parameters, but only a couple are required. If not specified, will assume default 
//---------------------------------------------------------------------------------------------------------------------------------------
//drivePID(Target, Maxspeed, Timeout, chainValue, When To Start Slow down, Error Width, Time Within Error, Min Speed, Trigger Distance, Speed)
//
//            drivePID(1000, 127, 5000, 0, 700, 15, 50, 30, -1, 0);       (10 Parameters MAX)
//            drivePID(1000, 127, 5000, 0, 700);                          (5 Parameters REQUIRED)
//---------------------------------------------------------------------------------------------------------------------------------------
//drivePID_distance(Target, Maxspeed, Timeout, Distance from Wall, 0 = Left Sensor/1 = Right Sensor, When To Start Slow Down, chainValue, Error Width, Time Within Error, Min Speed, Trigger Distance, Speed) 
//
//            drivePID_distance(1000, 127, 5000, 360, 0, 700, 0, 15, 50, 30, -1, 0);       (12 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//turnPID(Target, MaxSpeed, Timeout, Error Width, Time Within Error)
//
//            turnPID(90, 127, 5000, 15, 50);     (5 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//driveArcL(Target, radius, timeout, max Speed, chainValue, Error Width, Time in Error)
//
//            driveArcL(90, 500, 5000, 100, 0, 15, 50)      (7 Parameters)  
//            driveArcR(90, 500, 5000, 10, 0, 15, 50)      (7 Parameters)
//---------------------------------------------------------------------------------------------------------------------------------------
//chasSlow(speed, time), for like reseting in goal position
//
//            chasSlow(-50, 400);
//---------------------------------------------------------------------------------------------------------------------------------------
//Copy and Paste from Here
//  drivePID(-1000, 50, 3000, 0, 700, 15, 50, 40, -1, 0);
//  drivePID_distance(-1000, 50, 3000, 400, 0, 700, 0, 15, 50, 40, -1, 0); 
//  turnPID(90, 50, 3000, 1, 50);
//  driveArcR(90, 400, 5000, 100, 0, 15, 50);
//---------------------------------------------------------------------------------------------------------------------------------------

                      
  if (atn == 0) {
    //skills
    intake.move(127);
    descore.set_value(true);
    drivePID(620, 127, 1500, 0, 200, 10, 50, 80, -1, 0);
    turnPID(-105, 75, 1000, 1.6, 30, 0); 
    drivePID(-580, 60, 1500, 0, 350, 10, 50, 30, -1, 0);
    LF.move(-20);
    LM.move(-20);
    LB.move(-20);
    RF.move(-20);
    RM.move(-20);
    RB.move(-20);
    blocker.set_value(false);
    Lever.move(40);
    pros::delay(700);
    drivePID(1380, 127, 1500, 0, 600, 10, 50, 50, -1, 0);
    blocker.set_value(true);
    intake.move(-127);
    Lever.move(-40);
    lift.set_value(true);
    turnPID(210, 75, 1000, 1.6, 30, 0); 
    intake.move(127);
    scraper.set_value(true);
    drivePID_distance(900, 40, 1500, 480, 1, 600, 0, 15, 50, 40, -1, 0); 
    pros::delay(800);
    //scuffed
    drivePID_distance(-4000, 70, 3000, 150, 1, 2000, -100, 15, 50, 40, -1, 0);
  
    //non scuffed version
    // drivePID_distance(-200, 40, 1500, 480, 1, 100, -100, 15, 50, 40, -1, 0); 
    // turnPID(185, 75, 1000, 1.6, 30, -2);
    // drivePID(-400, 75, 1000, -100, 200, 10, 50, 50, -1, 0);
    // turnPID(210, 75, 1000, 1.6, 30, 2);
    // drivePID_distance(-2300, 127, 2000, 150, 1, 1200, -100, 15, 50, 40, -1, 0);

    driveArcL(-45, 200, 1500, 40, 50, 15, 50);
    drivePID(-200, 60, 1500, 0, 600, 10, 50, 50, -1, 0);
    turnPID(30, 75, 1000, 1.6, 30, 0);
    drivePID_distance(-600, 60, 1000, 460, 0, 599, -100, 15, 50, 40, -1, 0);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 600, 100, 10, 550, 50);
    pros::delay(100);
    Lever.move(-65);
    drivePID_distance(1400, 40, 1500, 480, 0, 675, 0, 15, 50, 40, -1, 0);
    Lever.move(0);
    pros::delay(800);
    drivePID_distance(-1350, 75, 1500, 460, 0, 300, -200, 15, 50, 60, -1, 0);
    scraper.set_value(false);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 200, 10, 550, 50);
    pros::delay(200);
    Lever.move(-40);
    blocker.set_value(true);
    drivePID_distance(400, 40, 1000, 460, 0, 200, 50, 15, 50, 30, -1, 0);
    Lever.move(0);
    driveArcR(87, 350, 1500, 30, 50, 15, 50);
    drivePID_distance(2500, 50, 2000, 5, 0, 1200, 0, 15, 50, 40, -1, 0);
    drivePID_distance(-500, 50, 500, 5, 0, 200, 0, 15, 50, 40, -1, 0);
    drivePID_distance(500, 50, 500, 5, 0, 200, 0, 15, 50, 40, -1, 0);
    turnPID(120, 75, 30, 1.6, 30, 0);
    driveArcR(-90, 350, 1000, 15, 0, 15, 50);
    turnPID(30, 75, 1000, 1.6, 30, 0);
    drivePID_distance(-500, 50, 500, 690, 1, 200, 0, 15, 50, 40, -1, 0);
    turnPID(75, 75, 1000, 1.6, 30, 0);
    lift.set_value(false);
    drivePID(-800, 40, 1000, 0, 400, 10, 50, 50, -1, 0);
    blocker.set_value(false);
    leverPID(750, 25, 1500, 200, 10, 550, 25);
    pros::delay(400);
    Lever.move(-40);
    intake.move(-127);
    drivePID(1380, 127, 1500, 0, 600, 10, 50, 50, -1, 0);
    blocker.set_value(true);
    Lever.move(-40);
    lift.set_value(true);
    scraper.set_value(true);
    turnPID(30, 75, 1000, 1.6, 30, 0);
    intake.move(127);
    drivePID_distance(900, 40, 1500, 480, 1, 600, 0, 15, 50, 40, -1, 0); 
    pros::delay(800);
    //scuffed
    drivePID_distance(-4000, 70, 3000, 150, 1, 2000, -100, 15, 50, 40, -1, 0);
    driveArcL(-45, 250, 1500, 50, 0, 15, 50);
    drivePID(-200, 60, 1500, 0, 600, 10, 50, 50, -1, 0);
    turnPID(210, 75, 1000, 1.6, 30, 0);
    drivePID_distance(-600, 60, 1000, 460, 0, 599, -100, 15, 50, 40, -1, 0);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 600, 100, 10, 550, 50);
    pros::delay(100);
    Lever.move(-65);
    drivePID_distance(1400, 40, 1500, 480, 0, 675, 0, 15, 50, 40, -1, 0);
    Lever.move(0);
    pros::delay(800);
    drivePID_distance(-1350, 75, 1500, 460, 0, 300, -200, 15, 50, 60, -1, 0);
    scraper.set_value(false);
    drivePID_distance(400, 40, 1000, 460, 0, 200, 50, 15, 50, 30, -1, 0);
    Lever.move(0);
    driveArcR(87, 350, 1500, 30, 50, 15, 50);
    scraper.set_value(true);
    drivePID_distance(1500, 50, 2000, 5, 0, 1200, 0, 15, 50, 40, -1, 0);
    scraper.set_value(false);
  

     
  } 
 

  else if (atn == 1) {
    //6+3 left

    intake.move(127);
    drivePID(600, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcL(29, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(820, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    pros::delay(30);
    turnPID(3, 75, 1000, 1.6, 30, 3); 
    drivePID(-450, 127, 800, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcL(-135, 120, 1000, 50, 0, 15, 50);
    turnPID(210, 75, 1000, 1.6, 30, 3); 
    drivePID_distance(-600, 127, 1000, 460, 1, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 127, 1000, 100, 10, 550, 100);
    Lever.move(50);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1000, 55, 1500, 480, 1, 675, 200, 15, 50, 30, -1, 0);
    chasSlow(20, 500);
    Lever.move(0);
    drivePID_distance(-160, 60, 600, 480, 1, 100, -50, 15, 50, 50, -1, 0);
    turnPID(-105, 75, 700, 1.6, 100, 2); 
    scraper.set_value(false);
    lift.set_value(false); 
    drivePID(-2300, 127, 1200, 0, 600, 10, 50, 20, -1, 0);
    LF.move(-10);
    LM.move(-10);
    LB.move(-10);
    RF.move(-10);
    RM.move(-10);
    RB.move(-10);
    blocker.set_value(false);
    leverPID(750, 60, 1300, 200, 10, 200, 50);
    Lever.move(-65);
    drivePID(1150, 127, 1000, 0, 350, 50, 50, 20, -1, 0);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-50);
    lift.set_value(true);
    turnPID(210, 75, 1000, 4, 10, 0);
    drivePID(-600, 127, 800, -200, 350, 10, 50, 20, -1, 0);
    Lever.move(0);
    turnPID(245, 75, 2000, 1.6, 1000, 0); 

    


  }


  else if (atn == 2) { 
   //6+3 right

    intake.move(127);
    drivePID(800, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcR(26, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(670, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    drivePID(-200, 127, 500, -50, 399, 10, 50, 127, -1, 0);
    turnPID(-32, 75, 700, 1.6, 30, -3); 
    drivePID(-300, 127, 1000, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcR(-135, 50, 1000, 50, 0, 15, 50);
    turnPID(150, 75, 600, 4, 10, 0);  
    drivePID_distance(-600, 127, 800, 440, 0, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 80);
    Lever.move(70);
    pros::delay(250);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1350, 60, 1600, 470, 0, 675, 0, 15, 50, 30, -1, 0);
    blocker.set_value(true);
    Lever.move(0);
    drivePID_distance(-100, 127, 500, 470, 0, 300, -200, 15, 50, 60, -1, 0); 
    scraper.set_value(false);
    turnPID(289, 75, 1000, 1, 70, 0);
    drivePID(1900, 127, 1300, 0, 600, 50, 10, 90, -1, 0);
    intake.move(-90);
    drivePID(-120, 127, 400, 0, 600, 50, 10, 90, -1, 0);
    pros::delay(550);
    drivePID(-1080, 127, 1800, 0, 350, 50, 50, 20, -1, 0);
    turnPID(-30, 75, 1000, 3, 10, 0); 
    Lever.move(0);
    drivePID(770, 127, 800, 200, 350, 10, 50, 20, -1, 0);
    turnPID(-50, 75, 10000, 1.6, 9999, 0); 
    intake.move(0);



    
  } 

  else if (atn == 3 ){
   //4 left

   intake.move(127);
    lift.set_value(true);
    descore.set_value(true);
    drivePID(600, 127, 1500, 200, 200, 10, 50, 80, -1, 0);
    scraper.set_value(true);
    turnPID(-90, 75, 1000, 1.6, 30, -3); 
    scraper.set_value(false);
    drivePID(800, 200, 1500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(210, 75, 1000, 1.6, 30, -3); 
    drivePID_distance(-600, 127, 500, 460, 1, 599, 200, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 600, 100, 10, 550, 50);
    turnPID(170, 75, 300, 1.6, 30, -3); 
    drivePID(200, 100, 500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(208, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(-750, 127, 800, -200, 350, 10, 50, 127, -1, 0);
    turnPID(235, 75, 1000, 1.6, 10, 0); 
    blocker.set_value(true);
    Lever.move(-65);
    pros::delay(1000);
    Lever.move(0);
    
  } 


  else if(atn == 4) {
    //4 right

    intake.move(127);
    lift.set_value(true);
    drivePID(600, 127, 1500, 200, 200, 10, 50, 80, -1, 0);
    scraper.set_value(true);
    turnPID(90, 75, 500, 1.6, 30, 3); 
    scraper.set_value(false);
    drivePID(950, 127, 1500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(150, 75, 400, 1.6, 30, 3); 
    drivePID_distance(-600, 127, 500, 440, 0, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 600, 100, 10, 550, 50);
    turnPID(103, 75, 300, 1.6, 30, -3); 
    drivePID(180, 100, 500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(143, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(-750, 127, 800, -200, 350, 10, 50, 127, -1, 0);
    turnPID(173, 75, 1000, 1.6, 10, 0); 
    scraper.set_value(true);
    blocker.set_value(true);
    Lever.move(-40);
    pros::delay(1500);
    Lever.move(0);


  } 

  else if (atn == 5){
    //3+4 left

    intake.move(127);
    lift.set_value(true);
    descore.set_value(true);
    drivePID(600, 127, 1500, 200, 200, 10, 50, 80, -1, 0);
    scraper.set_value(true);
    turnPID(-90, 75, 1000, 1.6, 30, -3); 
    scraper.set_value(false);
    drivePID(900, 200, 1500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(210, 75, 1000, 1.6, 30, -3); 
    drivePID_distance(-600, 127, 500, 460, 1, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 127, 600, 100, 10, 550, 100);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1000, 55, 1500, 480, 1, 675, 200, 15, 50, 30, -1, 0);
    chasSlow(20, 500);
    Lever.move(0);
    drivePID_distance(-160, 60, 600, 480, 1, 100, -50, 15, 50, 50, -1, 0);
    turnPID(-105, 75, 700, 1.6, 100, 2); 
    scraper.set_value(false);
    lift.set_value(false); 
    drivePID(-2300, 127, 1200, 0, 600, 10, 50, 20, -1, 0);
    LF.move(-10);
    LM.move(-10);
    LB.move(-10);
    RF.move(-10);
    RM.move(-10);
    RB.move(-10);
    blocker.set_value(false);
    leverPID(750, 50, 1300, 200, 10, 200, 50);
    Lever.move(-65);
    drivePID(1150, 127, 1000, 0, 350, 50, 50, 20, -1, 0);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-50);
    lift.set_value(true);
    descore.set_value(false);
    turnPID(210, 75, 1000, 4, 10, 0);
    drivePID(-600, 127, 800, -200, 350, 10, 50, 20, -1, 0);
    turnPID(245, 75, 2000, 1.6, 1000, 0); 
    Lever.move(0);


    
  
  }
  else if (atn == 6){
    //3+4 right
    intake.move(127);
    lift.set_value(true);
    drivePID(600, 127, 1500, 200, 500, 10, 50, 60, -1, 0);
    scraper.set_value(true);
    turnPID(90, 75, 500, 1.6, 30, 2); 
    scraper.set_value(false);
    drivePID(800, 127, 1500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(150, 75, 400, 1.6, 30, 2); 
    drivePID_distance(-600, 127, 500, 440, 0, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 80);
    Lever.move(70);
    pros::delay(250);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1350, 60, 1600, 470, 0, 675, 0, 15, 50, 30, -1, 0);
    blocker.set_value(true);
    Lever.move(0);
    drivePID_distance(-100, 127, 500, 470, 0, 300, -200, 15, 50, 60, -1, 0); 
    scraper.set_value(false);
    turnPID(289, 75, 1000, .5, 70, 0);
    drivePID(1900, 127, 1300, 0, 600, 50, 10, 90, -1, 0);
    intake.move(-90);
    drivePID(-120, 127, 400, 0, 600, 50, 10, 90, -1, 0);
    pros::delay(550);
    drivePID(-1080, 127, 1800, 0, 350, 50, 50, 20, -1, 0);
    turnPID(-30, 75, 1000, 3, 10, 0); 
    Lever.move(0);
    drivePID(770, 127, 800, 200, 350, 10, 50, 20, -1, 0);
    turnPID(-50, 75, 10000, 1.6, 9999, 0); 
    intake.move(0);

  
    // blocker.set_value(false);
    // leverPID(750, 115, 600, 100, 10, 550, 50);
    // blocker.set_value(true);
    // Lever.move(-65);
    // scraper.set_value(true);
    // drivePID_distance(1350, 60, 1900, 600, 0, 675, 0, 15, 50, 30, -1, 0);
    // Lever.move(0);
    // drivePID_distance(-100, 127, 500, 600, 0, 300, -200, 15, 50, 60, -1, 0); 
    // scraper.set_value(false);
    // turnPID(286, 75, 900, 1.6, 40, 0);
    // drivePID(2100, 127, 1300, 0, 600, 50, 10, 90, -1, 0);
    // intake.move(-127);
    // pros::delay(1000);
    // drivePID(-1250, 127, 1000, 0, 350, 50, 50, 20, -1, 0);
    // turnPID(-30, 75, 1000, 3, 10, 0); 
    // Lever.move(0);
    // drivePID(850, 127, 800, 200, 350, 10, 50, 20, -1, 0);
    // turnPID(-50, 75, 10000, 1.6, 9999, 0); 
    // intake.move(0);




  } 
  

  else if (atn == 7){
    //9 left

    intake.move(127);
    drivePID(800, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcL(28, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(600, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    pros::delay(30);
    turnPID(6, 75, 1000, 1.6, 30, 3); 
    drivePID(-450, 127, 800, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcL(-135, 90, 1000, 50, 0, 15, 50);
    turnPID(210, 75, 1000, 1.6, 30, 3); 
    drivePID_distance(-600, 127, 1000, 460, 1, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 100, 1000, 100, 10, 550, 50);
    pros::delay(100);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1400, 60, 1500, 480, 1, 675, 0, 15, 50, 30, -1, 0);
    Lever.move(0);
    drivePID_distance(-1350, 127, 1000, 480, 1, 300, -200, 15, 50, 60, -1, 0);
    scraper.set_value(false);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 200, 10, 550, 50);
    turnPID(170, 75, 300, 1.6, 30, -3); 
    drivePID(230, 100, 500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(208, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(-750, 127, 800, -200, 350, 10, 50, 127, -1, 0);
    turnPID(235, 75, 1000, 1.6, 10, 0);
    blocker.set_value(true);
    Lever.move(-65);
    pros::delay(1000);
    Lever.move(0);


  } 

  else if (atn == 8){
    //9 right   
    
    intake.move(127);
    drivePID(800, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcR(22, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(660, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    drivePID(-200, 127, 500, -50, 399, 10, 50, 127, -1, 0);
    turnPID(-35, 75, 700, 1.6, 30, -3); 
    drivePID(-300, 127, 1000, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcR(-135, 80, 1000, 50, 0, 15, 50);
    turnPID(150, 75, 600, 4, 10, 0);  
    drivePID_distance(-600, 127, 800, 440, 0, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 50);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1350, 60, 1900, 472, 0, 675, 0, 15, 50, 30, -1, 0);
    Lever.move(0);
    drivePID_distance(-1350, 127, 1200, 472, 0, 300, -200, 15, 50, 60, -1, 0);
    scraper.set_value(false); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 50);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-65);
    turnPID(103, 75, 300, 1.6, 30, -3); 
    drivePID(210, 100, 500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(143, 75, 500, 1.6, 30, 3); 
    Lever.move(0);
    descore.set_value(false);
    drivePID(-750, 127, 800, -200, 350, 10, 50, 127, -1, 0);
    turnPID(173, 75, 1000, 1.6, 10, 0); 
    blocker.set_value(true);
    scraper.set_value(true);
    
    

  }

  else if (atn == 9){
    //7 left


  }

  else if (atn == 10){
    //7 right
    

    intake.move(127);
    lift.set_value(true);
    descore.set_value(true);
    drivePID(620, 127, 1500, 200, 200, 10, 50, 80, -1, 0);
    scraper.set_value(true);
    turnPID(90, 75, 1000, 1.6, 30, 3); 
    scraper.set_value(false);
    drivePID(800, 200, 1500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(143, 75, 400, 1.6, 30, 3); 
    //4 ball variation
    drivePID_distance(-600, 127, 500, 440, 0, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 50);
    scraper.set_value(true);
    Lever.move(-65);
    drivePID_distance(1000, 100, 1000, 440, 0, 599, 0, 15, 50, 40, -1, 0); 
    delay(200);
    drivePID_distance(-600, 127, 500, 440, 0, 599, 0, 15, 50, 65, -1, 0);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 85, 600, 100, 10, 550, 50);
    
  }


  else if (atn == 11){
    // 6 left
    intake.move(127);
    drivePID(800, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcL(28, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(600, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    pros::delay(30);
    turnPID(6, 75, 1000, 1.6, 30, 3); 
    drivePID(-450, 127, 800, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcL(-135, 90, 1000, 50, 0, 15, 50);
    turnPID(210, 75, 1000, 1.6, 30, 3); 
    drivePID_distance(-600, 127, 1000, 460, 1, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 600, 100, 10, 550, 50);
    turnPID(170, 75, 300, 1.6, 30, -3); 
    drivePID(230, 100, 500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(208, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(-750, 127, 800, -200, 350, 10, 50, 127, -1, 0);
    turnPID(235, 75, 1000, 1.6, 10, 0); 
    blocker.set_value(true);
    Lever.move(-65);
    pros::delay(1000);
    Lever.move(0);
    
  
    
  }


  else if (atn == 12){
    // 6 right

    intake.move(127);
    drivePID(800, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcR(22, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(660, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    drivePID(-200, 127, 500, -50, 399, 10, 50, 127, -1, 0);
    turnPID(-35, 75, 700, 1.6, 30, -3); 
    drivePID(-300, 127, 1000, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcR(-135, 80, 1000, 50, 0, 15, 50);
    turnPID(150, 75, 600, 4, 10, 0);  
    drivePID_distance(-600, 127, 800, 440, 0, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 50);
    turnPID(103, 75, 300, 1.6, 30, -3); 
    drivePID(230, 100, 500, 200, 350, 10, 50, 100, -1, 0);
    turnPID(143, 75, 500, 1.6, 30, 3); 
    descore.set_value(false);
    drivePID(-750, 127, 800, -200, 350, 10, 50, 127, -1, 0);
    turnPID(173, 75, 1000, 1.6, 10, 0); 
    blocker.set_value(true);
    Lever.move(-65);
    pros::delay(1000);
    Lever.move(0);

    
  }


  else if (atn == 13){
    //sawp
    intake.move(127);
    drivePID_distance(200, 127, 500, 457, 0, 260, 50, 15, 50, 65, -1, 0); 
    drivePID_distance(-1770, 127, 1000, 457, 0, 800, 0, 15, 50, 65, -1, 0); 
    turnPID(-90, 75, 700, 1.6, 30, -2); 
    scraper.set_value(true);
    lift.set_value(true);
    pros::delay(100);
    drivePID_distance(480, 50, 800, 472, 0, 400, 0, 15, 50, 10, -1, 0);
    chasSlow(20, 350);
    drivePID_distance(-800, 127, 800, 435, 0, 600, -200, 15, 50, 60, -1, 0);
    scraper.set_value(false);
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 127, 900, 100, 10, 550, 127);
    intake.move(-127);
    LF.move(30);
    LM.move(30);
    LB.move(30);
    RF.move(-100);
    RM.move(-100);
    RB.move(-100);
    pros::delay(200);
    blocker.set_value(true);
    Lever.move(-60);
    turnPID(10, 75, 600, 1, 30, 0); 
    Lever.move(0);
    intake.move(127);
    driveArcL(10, 1500, 1000, 50, 50, 15, 50);
    descore.set_value(true);
    drivePID_distance(1400, 127, 1500, 1069, 0, 1300, 200, 15, 50, 127, -1, 0);
    scraper.set_value(true);
    turnPID(-40, 75, 700, 1.6, 30, -3);
    scraper.set_value(false);
    drivePID(1150, 127, 1000, 0, 500, 15, 50, 127, -1, 0);
    turnPID(-90, 75, 700, 1.6, 30, -2); 
    drivePID_distance(-600, 127, 700, 460, 1, 599, -50, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 127, 1200, 100, 10, 550, 127);
    intake.move(-127);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    intake.move(127);
    drivePID_distance(1000, 55, 1500, 480, 1, 675, 200, 15, 50, 30, -1, 0);
    chasSlow(20, 500);
    Lever.move(0);
    drivePID_distance(-160, 60, 600, 480, 1, 100, -50, 15, 50, 50, -1, 0);
    turnPID(-45, 75, 700, 1.6, 100, 2); 
    scraper.set_value(false);
    lift.set_value(false); 
    drivePID(-2300, 127, 1200, 0, 600, 10, 50, 20, -1, 0);
    LF.move(-10);
    LM.move(-10);
    LB.move(-10);
    RF.move(-10);
    RM.move(-10);
    RB.move(-10);
    blocker.set_value(false);
    leverPID(750, 50, 1300, 200, 10, 200, 50);
    Lever.move(-65);
    
  }


  else if (atn == 14){
intake.move(127);
    drivePID(600, 53, 1500, 200, 500, 10, 50, 50, -1, 0);
    scraper.set_value(true);
    driveArcL(26, 200, 1500, 50, 200, 15, 50);
    scraper.set_value(false);
    drivePID(800, 100, 1500, 200, 350, 10, 50, 20, -1, 0);
    pros::delay(30);
    turnPID(6, 75, 1000, 1.6, 30, 3); 
    drivePID(-450, 127, 800, -200, 700, 10, 50, 50, -1, 0);
    lift.set_value(true);
    driveArcL(-135, 110, 1000, 50, 0, 15, 50);
    turnPID(210, 75, 1000, 1.6, 30, 3); 
    drivePID_distance(-600, 127, 1000, 460, 1, 599, 0, 15, 50, 65, -1, 0); 
    LF.move(-80);
    LM.move(-80);
    LB.move(-80);
    RF.move(-80);
    RM.move(-80);
    RB.move(-80);
    blocker.set_value(false);
    leverPID(750, 115, 1000, 100, 10, 550, 50);
    Lever.move(50);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-65);
    scraper.set_value(true);
    drivePID_distance(1350, 50, 1600, 500, 1, 675, 0, 15, 50, 30, -1, 0);
    pros::delay(100);
    Lever.move(0);
    drivePID_distance(-125, 127, 5000, 480, 1, 300, -200, 15, 50, 60, -1, 0);
    scraper.set_value(false);
    lift.set_value(false); 
    turnPID(-106, 75, 1000, 1.6, 100, 0);

    drivePID(-2300, 127, 1100, 0, 600, 10, 50, 20, -1, 0);
    LF.move(-10);
    LM.move(-10);
    LB.move(-10);
    RF.move(-10);
    RM.move(-10);
    RB.move(-10);
    blocker.set_value(false);
    leverPID(750, 70, 1000, 200, 10, 550, 50);
    drivePID(400, 40, 1000, 0, 100, 50, 50, 20, -1, 0);
    pros::delay(100);
    blocker.set_value(true);
    Lever.move(-50);
    lift.set_value(true);
    drivePID(-600, 80, 1000, 0, 100, 50, 50, 20, -1, 0);
  }


  else if (atn == 15){

    
  }


  else if (atn == 16){
    
  }


  else if (atn == 17){
    
  }

  
  else if (atn == 18){

  }
    
  




}

