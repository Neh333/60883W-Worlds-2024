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
      else{iter++;}
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
 intakePis.set_value(true);
 pros::delay(100);
 imu.set_heading(318);

 intake.move_voltage(-12000);
 
 moveDriveTrain(6000, 0.1);
 wingPis.set_value(true);
 pros::delay(300);
 wingPis.set_value(false);
 
 drive.setPID(5);
 
 drive.addErrorFunc(8, LAMBDA(drive.setMaxVelocity(95)));
 drive.swerve(forwardRight, 22, imuTarget(360),2,35,100);

 intake.move_voltage(12000);
 
 moveDriveTrain(-12000, 0.1);
 
 /*
 drive.setPID(1);
 drive.move(right, imuTarget(90), 1, 70);

 drive.move(forward, 22, 1, 100);

 drive.move(right, imuTarget(180), 1, 70);

 drive.move(backward, 30, 1, 100);

 drive.move(right, imuTarget(273), 1, 70);

 wingPis.set_value(true);

 pros::delay(50);

 drive.move(backward, 22, 1, 100);

 moveDriveTrain(-12000, 0.2);

 wingPis.set_value(false);

 drive.move(forward, 24, 1, 100);

 drive.move(left, imuTarget(180), 1, 70);

 drive.move(forward, 42, 2, 100);

 intake.move_voltage(12000);

 drive.move(left, imuTarget(90), 1, 70);

 drive.move(forward, 30, 1, 100);
 */

 runOnError.remove();
 onErrorVector.clear();
}

void far(){
 
 
}

void closeBarTouch(){}

void HighFar(){
 pros::Task runOnError(onError_fn);

 intakePis.set_value(true);
 intake.move_voltage(-12000);

 pros::delay(200);

 //get first green
 moveDriveTrain(12000, 0.2);

 drive.setPID(3);
 drive.addErrorFunc(43, LAMBDA(wingPis.set_value(true)));
 drive.addErrorFunc(43, LAMBDA(drive.setMaxVelocity(35)));
 drive.addErrorFunc(43, LAMBDA(drive.setMaxTurnVelocity(95)));
 drive.addErrorFunc(22, LAMBDA(wingPis.set_value(false)));
 drive.addErrorFunc(22, LAMBDA(drive.setMaxVelocity(100)));
 drive.addErrorFunc(22, LAMBDA(drive.setMaxTurnVelocity(30)));

 drive.swerve(backwardLeft, 68, imuTarget(280), 2, 100, 30);
 
 moveDriveTrain(12000, 0.2);

 drive.setPID(1);
 drive.move(right, imuTarget(100), 2, 70);

 intake.move_voltage(12000);
 moveDriveTrain(12000, 0.4);

 intake.move_voltage(12000);
 moveDriveTrain(-12000, 0.4);
 
 drive.setPID(2);
 drive.move(left, imuTarget(37), 2, 70);
 intake.move_voltage(-12000);

 drive.setPID(5);
 drive.addErrorFunc(40, LAMBDA(drive.setMaxTurnVelocity(40)));
 drive.swerve(forwardRight, 56, imuTarget(50), 2, 100, 30);

 pros::delay(2500);
  
 /*
 drive.setPID(1);
 drive.move(left, imuTarget(0), 1, 70);

 drive.move(backward, 32, 1, 100);

 moveDriveTrain(12000, 0.2);

 drive.move(left, 180, 1, 70);

 moveDriveTrain(12000, 0.2);

 moveDriveTrain(-12000, 0.2);
 */
 

 runOnError.remove();
 onErrorVector.clear();

}

void closeRush(){}
void farRush(){}

void skills(){
 pros::Task runOnError (onError_fn);

 //sec1
 intakePis.set_value(true);
 imu.set_heading(318);
 drive.setPID(4);
 drive.swerve(backwardRight, 28, imuTarget(258), 1, 55, 100);
 
 drive.setPID(2);
 drive.move(forward, 6, 1, 70);
 drive.move(left, imuTarget(310), 1, 70);

 drive.setPID(1);
 drive.move(forward, 16, 1, 80);

 drive.move(right, imuTarget(65), 1, 70); 
 
 drive.setPID(2);
 drive.move(forward, 6, 1, 13);

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
 moveDriveTrain(-12000, 0.5);
 moveDriveTrain(12000, 0.2);
 moveDriveTrain(-12000, 0.5);
  
 runOnError.remove();
 onErrorVector.clear();
}

void nothing(){}