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
  drive.setScheduledConstants(PIDConstants[2]);
  drive.setScheduleThreshold_a(30);
  drive.setScheduleThreshold_l(10);

  //start running intake and drop it
  intake.move_voltage(-12000);
  hang.move_voltage(-12000);

  //let wing extend 
  pros::delay(100);

  drive.addErrorFunc(49, LAMBDA(hang.move_voltage(0)));
  drive.addErrorFunc(49, LAMBDA(frontWings.set_value(false)));
  drive.move(forward, 51, 3, 100);
  
  //turn to push over 
  drive.addErrorFunc(75, LAMBDA(intake.move_voltage(0)));
  drive.turn(right, imuTarget(95), 1, 70);

  frontWings.set_value(true);
  
  //push over
  drive.move(forward, 23, 1, 100);

  frontWings.set_value(false);
  
  //back up from mid bar
  drive.setPID(2);
  drive.setScheduleThreshold_l(NO_SCHEDULING);
  drive.move(backward, 4, 1, 100);

  //go to match laod bar
  drive.setPID(6);
  drive.swerve(backwardLeft, 57, imuTarget(27), 3,  60, 70);


  drive.setPID(1);
  drive.turn(right, imuTarget(125), 1, 70);
  

  //drop down backwings for removal 
  backWings.set_value(true);
 
  //remove match load from matchload zone
  drive.setPID(1);

  pros::delay(100);

  drive.turn(left, imuTarget(55), 1, 70);

  //bring up backwings
  backWings.set_value(false);
   
  //score pre load
  drive.setPID(4);
  drive.addErrorFunc(26, LAMBDA(drive.setMaxVelocity(90)));
  drive.addErrorFunc(26, LAMBDA(drive.setMaxTurnVelocity(70)));
  drive.swerve(backwardRight, 28, imuTarget(180), 2, 65, 90);

  //go touch match load bar and push over 
  intake.move_voltage(12000);
  
  drive.setPID(5);
  drive.swerve(forwardLeft, 69, imuTarget(90), 5,  90, 65);
  
  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear(); 

}

void far(){
  //fling pre load
  backWings.set_value(true);

  //run on error task
  pros::Task runOnError(onError_fn);
  
  //set scheduling attributes 
  drive.setScheduledConstants(PIDConstants[2]);
  drive.setScheduleThreshold_a(30);
  drive.setScheduleThreshold_l(10);

  //start running intake and drop it
  intake.move_voltage(-12000);
  hang.move_voltage(-12000);

  //let wing extend 
  pros::delay(80);
  
  //T
  drive.setPID(4); 
  drive.addErrorFunc(30, LAMBDA(hang.move_voltage(0)));
  drive.addErrorFunc(24, LAMBDA(backWings.set_value(false)));
  //drive.addErrorFunc(22, LAMBDA(drive.setMaxTurnVelocity(80)));
  drive.swerve(backwardLeft, 32, imuTarget(0), 2, 100, 70);

  //2nd pass over 
  drive.moveDriveTrain(-12000, 0.3);
  
  //get ball under elavation bar  
  drive.setPID(6);
  drive.addErrorFunc(50, LAMBDA(drive.setMaxVelocity(65)));
  drive.addErrorFunc(50, LAMBDA(drive.setMaxTurnVelocity(70)));
  drive.swerve(forwardRight, 70, imuTarget(91), 3, 70, 50);
  
  //go get safe ball
  drive.setPID(1);
  drive.addErrorFunc(23, LAMBDA(intake.move_voltage(-8000)));
  drive.move(backward, 26, 1, 100);
  
  //x-take triball
  drive.addErrorFunc(2, LAMBDA(intake.move_voltage(7000)));
  drive.turn(right, imuTarget(180), 1, 70);
  

  //get safeball
  drive.move(forward, 32, 2, 100);
  
  //revrese intake
  intake.move_voltage(-12000);

  drive.turn(left, imuTarget(93), 1, 70);

  drive.move(forward, 24, 2, 100);

  //go get autoline ball
  drive.setScheduleThreshold_l(NO_SCHEDULING);

  drive.setPID(2);
  drive.addErrorFunc(3, LAMBDA(intake.move_voltage(0)));
  drive.move(backward, 4, 1, 100);

  drive.setPID(1);

  //xtake safe ball 
  drive.turn(right, imuTarget(220), 1, 70);

  intake.move_voltage(8000);

  //line up to get auton ball on bar
  drive.setScheduleThreshold_l(10);
  drive.move(forward, 14, 1, 100);

  //reverse intake
  intake.move_voltage(-12000);

  drive.turn(left, imuTarget(131), 1, 70);

  //get auton ball on bar
  drive.move(forward, 20, 1, 100);

  //turn to corral for the score & open front wings 
  drive.addErrorFunc(2, LAMBDA(frontWings.set_value(true)));
  drive.addErrorFunc(1, LAMBDA(intake.move_voltage(10000)));
  drive.turn(right, imuTarget(260), 1, 70);
  
  //score all 4 triballs 
  drive.setPID(4);
  drive.swerve(forwardRight, 32, imuTarget(275), 2, 100, 95);
  drive.moveDriveTrain(12000, 0.3);

  pros::delay(80);

  //back up as to not be touching
  drive.moveDriveTrain(-12000, 0.2);
  
  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear();
 
}

void closeElims(){
  //fling pre load
  frontWings.set_value(true);
  //run on error task
  pros::Task runOnError(onError_fn);
  
  //set scheduling attributes 
  drive.setScheduledConstants(PIDConstants[2]);
  drive.setScheduleThreshold_a(30);
  drive.setScheduleThreshold_l(10);

  //start running intake and drop it
  intake.move_voltage(-12000);
  hang.move_voltage(-12000);

  //let wing extend 
  pros::delay(60);

  drive.addErrorFunc(49, LAMBDA(hang.move_voltage(0)));
  drive.addErrorFunc(49, LAMBDA(frontWings.set_value(false)));
  drive.move(forward, 51, 3, 100);
  
  //turn to push over 
  drive.addErrorFunc(75, LAMBDA(intake.move_voltage(0)));
  drive.turn(right, imuTarget(95), 1, 70);

  frontWings.set_value(true);
  
  //push over
  drive.move(forward, 23, 1, 100);

  frontWings.set_value(false);
  
  //back up from mid bar
  drive.setPID(2);
  drive.setScheduleThreshold_l(NO_SCHEDULING);
  drive.move(backward, 4, 1, 100);

  //go to match laod bar
  drive.setPID(6);
  drive.swerve(backwardLeft, 57, imuTarget(27), 3,  60, 70);


  drive.setPID(1);
  drive.turn(right, imuTarget(125), 1, 70);
  

  //drop down backwings for removal 
  backWings.set_value(true);
 
  //remove match load from matchload zone
  drive.setPID(1);

  pros::delay(100);

  drive.turn(left, imuTarget(55), 1, 70);

  //bring up backwings
  backWings.set_value(false);
   
  //score pre load
  drive.setPID(4);
  drive.addErrorFunc(26, LAMBDA(drive.setMaxVelocity(90)));
  drive.addErrorFunc(26, LAMBDA(drive.setMaxTurnVelocity(70)));
  drive.swerve(backwardRight, 28, imuTarget(180), 2, 65, 90);

  //go touch match load bar and push over 
  intake.move_voltage(12000);
  
  drive.setPID(5);
  drive.swerve(forwardLeft, 69, imuTarget(90), 5,  90, 65);
  
  //go back to touch bar
  drive.swerve(backwardRight, 36, imuTarget(115), 2, 100, 95);
  
  //drop wing down to touch bar
  backWings.set_value(false);

  //switch to toogle so wings will stay down once cp control runs
  backWingTog = false;
  
  //remove on error task and clear the on error vector
  runOnError.remove();
  onErrorVector.clear();
}

void farRush(){
 //fling pre load
 frontWings.set_value(true);
 
 //run on Error Task
 pros::Task runOnError (onError_fn);

 //set scheduling attributes 
 drive.setScheduledConstants(PIDConstants[2]);
 drive.setScheduleThreshold_a(30);

 //start running intake and drop it
 intake.move_voltage(-12000);
 hang.move_voltage(-12000);
 
 //let wing extend
 pros::delay(80);

 //get first bar green on auto line
 drive.setScheduleThreshold_l(58);
 drive.addErrorFunc(70, LAMBDA(hang.move_voltage(-12000)));
 drive.hardStop(forward, 48, 72, 100);

 //ensure not to "over take"
 drive.addErrorFunc(40, LAMBDA(intake.move_voltage(0)));
 
 //back up and maintain angle 
 drive.setPID(4);
 drive.swerve(backwardShortest, 58, 346, 2, 100, 95);
 
 //xtake auto line ball
 drive.setPID(1);
 drive.addErrorFunc(3, LAMBDA(intake.move_voltage(12000)));
 drive.move(right, imuTarget(90), 1, 70);
 
 //grab triball under elevation bar
 drive.move(left, imuTarget(270), 1, 70);

 intake.move_voltage(-12000);

 drive.hardStop(forward, 25, 60, 100);

 /*
 //hardstop into swerve to push into goal
 drive.hardStop(backward, 22, 60, 100);
 
 //goal push swerve 
 drive.setPID(4);
 drive.addErrorFunc(44, LAMBDA(wings.set_value(true)));
 drive.addErrorFunc(40, LAMBDA(drive.setMaxTurnVelocity(0)));
 drive.addErrorFunc(33, LAMBDA(drive.setMaxTurnVelocity(100)));
 drive.addErrorFunc(33, LAMBDA(drive.setMaxVelocity(59)));
 drive.addErrorFunc(22, LAMBDA(wings.set_value(false)));
 drive.swerve(backwardLeft, 48, imuTarget(185), 2, 100, 90);

 moveDriveTrain(-12000, 0.5);
 
 //set up for second push
 drive.setCustomPID(64, 148,  3,   0,  240,  240,   0);
 drive.swerve(forwardRight, 6, imuTarget(200), 1, 100, 95);
 
 //overturn for push 
 drive.setStandStill(turn, 9, 2);
 drive.addErrorFunc(3, LAMBDA(intake.move_voltage(12000)));

 drive.setPID(1);
 drive.move(left, 195, 2, 70);

 drive.setStandStill(turn, 7, 2);
 
 intakePis.set_value(false);

 //push
 moveDriveTrain(12000, 0.7);

 //back up to get safe ball
 drive.setPID(3);
 drive.addErrorFunc(10, LAMBDA(intakePis.set_value(true);));
 drive.swerve(backwardShortest, 16, 0, 2, 100, 95);
 


 drive.setPID(1);
 drive.move(left, imuTarget(285), 1, 70);

 intake.move_voltage(-12000);
  */
 
 //hardstop into swerve to score safe ball
                  /*{kP, kPt,  kI, kIt, kD,  kDt,  kPd}*/

 
  /*
 drive.setCustomPID(29, 0,    0,  0,   160, 0,  140);
 drive.hardStop(forward, 29, 72, 100);
 
 drive.setPID(4);
 drive.addErrorFunc(38, LAMBDA(drive.setMaxTurnVelocity(100)));
 drive.addErrorFunc(38, LAMBDA(drive.setMaxVelocity(95)));
 drive.addErrorFunc(10, LAMBDA(intake.move_voltage(12000)));
 drive.addErrorFunc(10, LAMBDA(intakePis.set_value(true)));

 drive.swerve(forwardRight, 62, imuTarget(90), 2, 100, 95);
 */
 
 runOnError.remove();
 onErrorVector.clear();
}

void nothing(){}