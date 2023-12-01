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
 drive.setPID(2);
 
 drive.move(forward, 12, 1, 70);
 pros::delay(1000);


}

void far(){

}

void closeBarTouch(){}

void HighClose(){}
void HighFar(){
 pros::Task runOnError(onError_fn);

 intakePis.set_value(true);
 intake.move_voltage(-12000);

 pros::delay(300);

 //get first green
 moveDriveTrain(12000, 0.2);

 drive.setPID(3);
 drive.addErrorFunc(39, LAMBDA(drive.setMaxVelocity(20)));
 drive.addErrorFunc(39, LAMBDA(drive.setMaxTurnVelocity(25)));
 drive.addErrorFunc(38, LAMBDA(wingPis.set_value(true)));
 //drive.addErrorFunc(25, LAMBDA(pros::delay(200)));
 drive.addErrorFunc(22, LAMBDA(wingPis.set_value(false)));
 drive.addErrorFunc(21.5, LAMBDA(drive.setMaxVelocity(40)));
 drive.addErrorFunc(21.5, LAMBDA(drive.setMaxTurnVelocity(100)));

 drive.swerve(backwardLeft, 67, imuTarget(275), 2, 100, 30);

 moveDriveTrain(-12000, 0.2);

 drive.setPID(1);
 drive.move(right, imuTarget(90), 2, 70);

 pros::delay(1000);

 intake.move_voltage(12000);
 moveDriveTrain(12000, 0.5);

 runOnError.remove();
 onErrorVector.clear();

}

void closeRush(){}
void farRush(){}

void skills(){
 //sec1
 imu.set_heading(320);
 drive.setPID(7);
 drive.swerve(backwardRight, 30, imuTarget(0), 1, 52, 100);
 pros::delay(1000);

 drive.setPID(4);
 drive.move(forward, 6, 1, 70);

 drive.setPID(1);
 drive.move(left, imuTarget(280), 1, 70);


 drive.setPID(1);
 drive.move(forward, 20, 1, 70);
 

 drive.setCustomPID(0, 56, 0, 0, 0, 101, 0);
 drive.move(right, imuTarget(64), 1, 70); 
 
 drive.setCustomPID(13, 0, 0, 0, 17, 0, 123);
 drive.move(forward, 16, 1, 100);

 
 //shoot match loads 
 puncher.move_voltage(10000);
 pros::delay(34000);
 puncher.move_voltage(0);

 
 puncherDown();

 //sec 2
 //go to other side 
 drive.setCustomPID(0, 83, 0, 0, 0, 57, 0);
 drive.move(right, imuTarget(125), 1, 100); //61 degree turn 

 
 drive.setPID(1);
 drive.move(backward, 18, 1, 100);
 
 
 drive.setPID(3);
 drive.move(left, imuTarget(92), 1, 70);

 intakePis.set_value(true);
 
       
 drive.move(backward, 72, 2, 70);
 
 //swerve to goal first push
 intakePis.set_value(false);

 
  drive.setPID(8); 
  drive.swerve(backwardLeft, 44, imuTarget(360), 2, 80, 100); //268 degree turn 

  moveDriveTrain(-12000, .6);

       
  drive.setPID(4);
  drive.move(forward, 14, 1,70);
  
  drive.setPID(1);
  drive.move(right, imuTarget(115), 1, 70);

  drive.setPID(1);
  drive.move(forward, 48, 1, 100);
  
  drive.setPID(3);
  drive.move(left, imuTarget(80), 1, 70);
  

  //2nd push
  wingPis.set_value(true);
  drive.setPID(1);
  drive.move(backward, 22, 1, 100);
  
  wingPis.set_value(false);

  drive.move(forward, 26, 1, 100);
  
  drive.setPID(3);
  drive.move(left,imuTarget(30),1,100);

  wingPis.set_value(true);
  pros::delay(200);
   
  
  //3rd push 
  drive.setPID(8);
  drive.swerve(backwardRight, 38, imuTarget(90), 1, 100, 95);
  moveDriveTrain(-12000, 1);

  

  /*  
  wingPis.set_value(false);             
  drive.setCustomPID(60,  110, 0,  60,   151, 177,  0);
  drive.swerve(forwardRight, 48, imuTarget(200), 2.5, 70, 100); 
  
  drive.setPID(1);
  drive.move(left, imuTarget(110), 1, 100);
  

  //4th push 
  drive.setPID(4);
  drive.swerve(backwardRight, 32, imuTarget(180), 1, 100, 60);
  moveDriveVoltage(-12000);
  */

}

void nothing(){}