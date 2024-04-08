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
  drive.turn(right, imuTarget(93), 1, 70);

  frontWings.set_value(true);
  
  //push over
  drive.move(forward, 23, 1, 100);

  frontWings.set_value(false);
  
  //back up from mid bar
  drive.setPID(2);
  drive.setScheduleThreshold_l(NO_SCHEDULING);
  drive.move(backward, 4, 1, 100);

  //line up for swerve
  drive.setPID(4);
  drive.swerve(backwardLeft, 50, imuTarget(27), 3,  80, 90);

  
  //go to match laod bar
  drive.setPID(2);
  drive.move(backward, 10, 1, 100);

  drive.setPID(1);
  drive.turn(right, imuTarget(125), 1, 70);

  drive.setPID(2);
  drive.move(backward, 4, 1, 100);

  //drop down backwings for removal 
  backWings.set_value(true);
 
  //remove match load from matchload zone
  drive.setPID(3);
  drive.setScheduleThreshold_a(NO_SCHEDULING);

  pros::delay(100);

  drive.turn(left, imuTarget(68), 1, 70);

  //bring up backwings
  backWings.set_value(false);
   
  //score pre load
  drive.setPID(4);
  drive.swerve(backwardRight, 28, imuTarget(180), 2, 90, 70);
  
  //2nd pass over 
  drive.moveDriveTrain(-12000, 0.3);

  pros::delay(400);

  drive.moveDriveTrain(-12000, 0.2);

  //go touch match load bar and push over 
  intake.move_voltage(12000);
  
  drive.setPID(5);
  drive.swerve(forwardLeft, 54, imuTarget(270), 4,  90, 73);
  
  drive.setPID(1);
  drive.setScheduleThreshold_l(10);
  drive.move(forward, 15, 2, 100);
  
  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear();
 
}

void far(){
 
}

void closeElims(){
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
  drive.turn(right, imuTarget(93), 1, 70);

  frontWings.set_value(true);
  
  //push over
  drive.move(forward, 23, 1, 100);

  frontWings.set_value(false);
  
  //back up from mid bar
  drive.setPID(2);
  drive.setScheduleThreshold_l(NO_SCHEDULING);
  drive.move(backward, 4, 1, 100);

  //line up for swerve
  drive.setPID(4);
  drive.swerve(backwardLeft, 50, imuTarget(27), 3,  80, 90);

  
  //go to match laod bar
  drive.setPID(2);
  drive.move(backward, 10, 1, 100);

  drive.setPID(1);
  drive.turn(right, imuTarget(125), 1, 70);

  drive.setPID(2);
  drive.move(backward, 4, 1, 100);

  //drop down backwings for removal 
  backWings.set_value(true);
 
  //remove match load from matchload zone
  drive.setPID(3);
  drive.setScheduleThreshold_a(NO_SCHEDULING);

  pros::delay(100);

  drive.turn(left, imuTarget(68), 1, 70);

  //bring up backwings
  backWings.set_value(false);
   
  //score pre load
  drive.setPID(4);
  drive.swerve(backwardRight, 28, imuTarget(180), 2, 90, 70);
  
  //2nd pass over 
  drive.moveDriveTrain(-12000, 0.3);

  pros::delay(400);

  drive.moveDriveTrain(-12000, 0.2);

  //go touch match load bar and push over 
  intake.move_voltage(12000);
  
  drive.setPID(5);
  drive.swerve(forwardLeft, 54, imuTarget(270), 4,  90, 75);
  
  drive.setPID(1);
  drive.setScheduleThreshold_l(10);
  drive.move(forward, 15, 2, 100);
  
  //go back to touch bar
  drive.move(backward, 34, 2, 100);
  
  //drop wing down to touch bar
  backWings.set_value(false);

  //switch to toogle so wings will stay down once cp control runs
  backWingTog = false;
  
  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear();
}

void farRush(){
 
}

void nothing(){}