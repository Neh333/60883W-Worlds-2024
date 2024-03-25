#include "auton.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "drive.hpp"


//object instances 
Drive drive(leftMotors, rightMotors, imu);
pros::Mutex onErrorMutex;
std::vector<errorFuncTuple> onErrorVector;

/* AUTON NOTES: 
*  run turn velo 70 or lower
*  run gen lat 100
*  make sure movment has a big enough timeout 
*  Use hardstops when possible 
*/
void close(){
 //drive.move(forward, 24, 1, 100);
 //drive.setScheduledConstants(PIDConstants[1]);
 //drive.setScheduleThreshold_a(50);

 drive.turn(right, 65, 1, 70);
 pros::delay(1000);
 
 /*
 drive.turn(right, 80, 1, 70);
 pros::delay(1000);

  drive.turn(right, 95, 1, 70);
  pros::delay(1000);

  drive.turn(right, 115, 2, 70);
  pros::delay(1000);

  drive.turn(right, 130, 2, 70);
  pros::delay(1000);

  drive.turn(right, 145, 2, 70);
  pros::delay(1000);

  drive.turn(right, 160, 2, 70);
  pros::delay(1000);

  drive.turn(right, 175, 2, 70);
  pros::delay(1000);

  drive.turn(right, 190, 2, 70);
  pros::delay(1000);

  drive.turn(right, 205, 2, 70);
  pros::delay(1000);

  drive.turn(right, 220, 2, 70);
  pros::delay(1000);

  drive.turn(right, 235, 2, 70);
  pros::delay(1000);

  drive.turn(right, 250, 2, 70);
  pros::delay(1000);
 */
 
}

void far(){
 
 
}

void closeElims(){
 
}

void farRush(){
 
}

void nothing(){}