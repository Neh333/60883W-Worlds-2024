#include "auton.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "drive.hpp"
#include "util.hpp"


//object instances 
Drive drive(leftMotors, rightMotors, imu);
pros::Mutex onErrorMutex;
std::vector<errorFuncTuple> onErrorVector;

/* AUTON NOTES: 
*  run turn velo 70 or lower
*  run gen lat 100
*  make sure movment has a big enough timeout 
*/
void close(){
  //fling pre load
  frontWings.set_value(true);
  //run on error task
  pros::Task runOnError(onError_fn);
  
  //set scheduling attributes 
  drive.setScheduledConstants(PIDConstants[1]);
  drive.setScheduleThreshold_a(30);
  drive.setScheduleThreshold_l(10);

  //start running intake and drop it
  intake.move_voltage(-12000);
  hang.move_voltage(-12000);

  //let wing extend 
  pros::delay(80);

  drive.addErrorFunc(48, LAMBDA(hang.move_voltage(0)));
  drive.addErrorFunc(48, LAMBDA(frontWings.set_value(false)));
  drive.move(forward, 50, 2, 100);
  
  //turn to push over 
  drive.turn(right, imuTarget(95), 1, 70);

  frontWings.set_value(true);
  
  //push over
  drive.move(forward, 23, 1, 100);

  frontWings.set_value(false);

  //line up for swerve
  drive.setPID(4);
  //drive.addErrorFunc(28, LAMBDA(drive.setMaxVelocity(100)));
  //drive.addErrorFunc(5, LAMBDA(drive.setMaxTurnVelocity(80)));
  drive.swerve(backwardLeft, 55, imuTarget(25), 3,  90, 100);

  //drive.hardStopSwerve(backwardLeft, 36, 72, imuTarget(30), 75, 100);
  
  //go to match laod bar
  drive.setPID(5);
  drive.swerve(backwardRight, 24, imuTarget(125), 2,  95, 100);
  
  //drop down backwings for removal 
  backWings.set_value(true);
 
  
  //remove match load from matchload zone
  drive.setPID(3);
  drive.setScheduleThreshold_a(NO_SCHEDULING);
  pros::delay(100);


  drive.turn(left, imuTarget(60), 1, 70);
  
  //bring up backwings
  backWings.set_value(false);
  

  /*
  //score pre load
  drive.setPID(4);
  drive.swerve(backwardRight, 28, imuTarget(0), 2, 100, 90);
  
  //2nd pass over 
  drive.moveDriveTrain(-12000, 0.3);

  //go touch match load bar and push over 
  intake.move_voltage(12000);
  */

  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear();
 
}

void far(){
 
}

void closeElims(){
 
}

void farRush(){
 
}

void nothing(){}