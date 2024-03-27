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
*/
void close(){
  //run on error task
  pros::Task runOnError(onError_fn);
  
  //set scheduling attributes 
  drive.setScheduledConstants(PIDConstants[1]);
  drive.setScheduleThreshold_a(30);
  drive.setScheduleThreshold_l(10);
  
  //fling pre load
  frontWings.set_value(true);

  //start running intake and drop it
  intake.move_voltage(-12000);
  hang.move_voltage(-12000);

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
  drive.hardStopSwerve(backwardLeft,  38, 72, imuTarget(50), 100, 95);
  
  //go to match laod bar
  drive.setPID(5);
  drive.swerve(backwardRight, 24, imuTarget(130), 2,  95, 100);

  backWings.set_value(true);

  //remove match load from matchload zone
  drive.setPID(3);
  drive.turn(left, imuTarget(85), 1, 70);
  
  //score pre load
  drive.setPID(5);
  drive.swerve(backwardRight, 48, imuTarget(0), 2, 95, 100);
  
  //2nd pass over 
  drive.moveDriveTrain(-12000, 0.3);

  //go touch match load bar and push over 
  intake.move_voltage(12000);

  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear();
 
}

void far(){

  drive.setPID(1);
  drive.setScheduledConstants(PIDConstants[1]);
  drive.setScheduleThreshold_l(10);
   
  drive.move(forward, 10, 2, 100);
  pros::delay(1000);

  drive.move(forward, 24, 2, 100);
  pros::delay(1000);
  
  drive.move(forward, 48, 2, 100);
  pros::delay(1000);
 
 
}

void closeElims(){
 
}

void farRush(){
 
}

void nothing(){}