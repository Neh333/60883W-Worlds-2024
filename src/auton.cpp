#include "auton.hpp"
#include "funcs.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "PID.hpp"
#include "onError.hpp"
Drive drive; //object instance of PID named drive 

pros::Mutex onErrorMutex;
std::vector<errorFuncTuple> onErrorVector;

void Drive::addErrorFunc(float onError, void input()){
    onErrorVector.emplace_back(errorFuncTuple(input, inchToTick(onError), false));
}

void onError_fn(void* param){
  std::uint32_t startTime = pros::millis();
  while(true){
  onErrorMutex.take();
  if(drive.getPIDStatus()){
    auto iter = onErrorVector.begin();
    while(iter != onErrorVector.end()){
      if (!iter->called && (iter->onError >= drive.getError())){
        iter->func();
        iter->called = true;
      }
      iter++;
      }
    }
    onErrorMutex.give();
    pros::Task::delay_until(&startTime, 10);  
  }
}

/* AUTON NOTES 
*  run turn velo 70 or lower
*  run gen lat 100
*  run short lat 70 or lower
*  make sure movment has a big enough timeout 
*/

void close(){   
 pros::Task runOnError(onError_fn);
 imu.set_heading(315);
 intake.move_voltage(-12000);
 intakePis.set_value(true);
 
 drive.setPID(4);
 drive.addErrorFunc(8, LAMBDA(drive.setMaxVelocity(95)));
 drive.swerve(forwardRight, 24, imuTarget(0),2,35,100);

 intake.move_voltage(12000);
 pros::delay(200);
 
 drive.addErrorFunc(30, LAMBDA(drive.setMaxVelocity(30)));
 drive.addErrorFunc(30, LAMBDA(drive.setMaxTurnVelocity(100)));
 drive.addErrorFunc(30, LAMBDA(wingPis.set_value(true)));
 drive.addErrorFunc(25, LAMBDA(drive.setMaxVelocity(100)));
 drive.addErrorFunc(25, LAMBDA(drive.setMaxTurnVelocity(60)));
 drive.addErrorFunc(8, LAMBDA(wingPis.set_value(false)));
 drive.setCustomPID(60, 92,  0,  0,  90, 120,   0);
 drive.swerve(backwardLeft, 40, imuTarget(275),2,100,70);
 
 drive.setPID(1);
 drive.move(left, 180, 2, 70);
 drive.move(forward, 24, 1, 100);

 runOnError.remove();
 onErrorVector.clear();
 
}

void far(){
 drive.setStandStill(lateral, 9, 2);

 imu.set_heading(41.5);
 pros::Task runOnError(onError_fn);
 intakePis.set_value(true);

 wingPis.set_value(true);
 pros::delay(50);

 drive.setPID(2);
 drive.move(backward, 8, 1, 70);
 drive.move(left, imuTarget(0), 1, 100);
 
 drive.addErrorFunc(10, LAMBDA(wingPis.set_value(false)));
 drive.setPID(1);
 drive.move(backward, 24, 1, 100);

 drive.setPID(2);
 drive.move(forward, 12, 1, 70);

 drive.setPID(1);
 drive.move(right, imuTarget(105), 1, 70);

 intake.move_voltage(-12000);

 drive.move(forward, 48, 5, 80);

 drive.setStandStill(turn, 9, 0.2);

 drive.move(right, imuTarget(240), 1, 70);
 
 drive.addErrorFunc(5, LAMBDA(intake.move_voltage(12000)));
 drive.move(forward, 32, 1, 80);

 moveDriveTrain(-12000, 0.3);
 
 drive.move(right, imuTarget(346), 1, 80);

 drive.move(forward, 44, 4, 100);

 drive.move(right, imuTarget(77), 1, 80);

 drive.move(forward, 28, 1, 100);

 runOnError.remove();
 onErrorVector.clear();

 
}

void closeBarTouch(){}

void HighFar(){
 drive.setStandStill(lateral, 9, 2);

 imu.set_heading(41.5);
 pros::Task runOnError(onError_fn);
 intakePis.set_value(true);

 wingPis.set_value(true);
 pros::delay(50);

 drive.setPID(2);
 drive.move(backward, 8, 1, 70);
 drive.move(left, imuTarget(0), 1, 100);
 
 drive.addErrorFunc(10, LAMBDA(wingPis.set_value(false)));
 drive.setPID(1);
 drive.move(backward, 24, 1, 100);

 drive.setPID(2);
 drive.move(forward, 12, 1, 70);

 drive.setPID(1);
 drive.move(right, imuTarget(105), 1, 70);

 intake.move_voltage(-12000);

 drive.move(forward, 48, 5, 80);

 drive.move(right, imuTarget(230), 1, 70);

 drive.setPID(2);
 drive.addErrorFunc(4, LAMBDA(intake.move_voltage(7000)));
 drive.move(forward, 12, 1, 80);

 intake.move_voltage(-12000);

 drive.setPID(1);
 drive.move(left, imuTarget(133), 1, 50);

 drive.move(forward, 18, 1, 100);

 drive.setPID(2);
 drive.move(left, imuTarget(90), 1, 50);

 wingPis.set_value(true);
 pros::delay(50);

 moveDriveTrain(-12000, 0.45);
 pros::delay(300);

 wingPis.set_value(false);

 drive.setPID(1);
 drive.move(left, 180, 2, 80);

 intake.move_voltage(12000);

 moveDriveTrain(12000, 0.45);
 moveDriveTrain(-12000, 0.3);

 runOnError.remove();
 onErrorVector.clear();

 /*
 pros::Task runOnError(onError_fn);
 
 //get first green
 intakePis.set_value(true);
 intake.move_voltage(-12000);
 pros::delay(100);
 moveDriveTrain(12000, 0.2);

 drive.setPID(3);
 drive.addErrorFunc(43, LAMBDA(wingPis.set_value(true)));
 drive.addErrorFunc(43, LAMBDA(drive.setMaxVelocity(35)));
 drive.addErrorFunc(43, LAMBDA(drive.setMaxTurnVelocity(95)));
 drive.addErrorFunc(22, LAMBDA(drive.setMaxVelocity(100)));
 drive.addErrorFunc(22, LAMBDA(drive.setMaxTurnVelocity(30)));
 drive.addErrorFunc(16, LAMBDA(wingPis.set_value(false)));

 drive.swerve(backwardLeft, 68, imuTarget(280), 2, 100, 25);
 
 moveDriveTrain(12000, 0.1);

 drive.setPID(1);
 drive.move(right, imuTarget(100), 2, 80);

 intake.move_voltage(12000);
 moveDriveTrain(12000, 0.55);
 
 drive.setPID(2);
 drive.move(backward, 12, 1, 70);

 //get first 1/3 green 
 drive.move(left, imuTarget(23), 2, 70);
 intake.move_voltage(-12000);
 wingPis.set_value(false);

 drive.setPID(1);
 drive.move(forward, 47, 5, 100);

 //dump tribal
 drive.move(right, imuTarget(141), 1, 70);

 drive.addErrorFunc(9, LAMBDA(intake.move_voltage(7000)));
 
 //get 2/3 tribal
 drive.setPID(2);
 drive.move(forward, 13, 1, 70);

 intake.move_voltage(-12000);

 drive.setPID(1);
 drive.move(left, imuTarget(35), 1, 70);

 drive.setPID(2);
 drive.move(forward, 18, 1, 70);
 
 drive.move(left, imuTarget(0), 1, 80);

 wingPis.set_value(true);
 pros::delay(50);

 moveDriveTrain(-12000, 1);
 wingPis.set_value(false);
 pros::delay(70);

 moveDriveTrain(12000, 0.09);

 drive.move(right, 180, 2, 90);

 intake.move_voltage(12000);

 moveDriveTrain(12000, 0.45);
 moveDriveTrain(-12000, 0.1);


 runOnError.remove();
 onErrorVector.clear();
 */

}

void closeRush(){}
void farRush(){}

void skills(){
 pros::Task runOnError (onError_fn);

 //sec1
 intakePis.set_value(true);
 imu.set_heading(318);
 drive.setPID(4);
 drive.swerve(backwardRight, 26, imuTarget(5), 2, 85, 100);
 
 drive.setPID(2);
 drive.move(forward, 4, 1, 70);
 drive.move(left, imuTarget(310), 1, 70);

 drive.setPID(1);
 drive.move(forward, 16, 1, 80);

 drive.move(right, imuTarget(65), 1, 70); 
 
 
 drive.setCustomPID(50,0,2,0,160,0,85);
 drive.move(forward, 4, 2, 20);


 //shoot match loads 
 puncher.move_voltage(10000);
 pros::delay(34000);
 puncher.move_voltage(0);
 
 puncher.move_voltage(8000);
 pros::delay(500);
 puncher.move_voltage(0);

 //sec 2
 //go to other side 
 drive.setPID(2);
 drive.move(right, imuTarget(125), 1, 100); 

 drive.move(backward, 14, 1, 100);

 drive.move(left, imuTarget(85), 1, 70);

 drive.setPID(1);
 drive.move(backward, 72, 2, 70);
 
 //swerve to goal first push
 drive.setPID(3); 
 drive.addErrorFunc(25, LAMBDA(drive.setMaxVelocity(45)));
 drive.swerve(backwardLeft, 50, imuTarget(360), 2, 75, 100); 

 moveDriveTrain(-12000, .5);

 drive.setPID(2);
 drive.move(forward, 12, 1,70);
  
 drive.setPID(1);
 drive.move(right, imuTarget(100), 1, 70);

 drive.move(forward, 48, 1, 100);
  
 drive.setPID(2);
 drive.move(left, imuTarget(60), 1, 70);
 
 //2nd push
 wingPis.set_value(true);
 drive.setPID(1);
 drive.move(backward, 26, 1, 100);
 moveDriveTrain(-12000, .5);

  
 wingPis.set_value(false);
  
 drive.setPID(2);
 drive.move(right, imuTarget(100), 1, 70);

 drive.setPID(1);
 drive.move(forward, 22, 1, 100);
  
 drive.setPID(1);
 drive.move(left,imuTarget(30),1,100);

 wingPis.set_value(true); 
   
 //3rd push 
 drive.setPID(3);

 drive.swerve(backwardRight, 38, imuTarget(90), 1, 90, 100);
 moveDriveTrain(-12000, 0.3);
 moveDriveTrain(12000, 0.1);
 moveDriveTrain(-12000, 0.3);
  
 runOnError.remove();
 onErrorVector.clear();
}

void nothing(){}