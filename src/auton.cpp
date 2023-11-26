#include "auton.hpp"
#include "funcs.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"
#include "PID.hpp"
Drive drive; //object instance of PID named drive 

/* AUTON NOTES 
*  run turn velo 70 or lower
*  run gen lat 100
*  run short lat 80 or lower
*  make sure movment has a big enough timeout 
*/

void close(){   
  drive.setPID(2);

  drive.move(right, 30, 1, 70);
  pros::delay(1000);
 
  drive.move(right, 45, 1, 70);
  pros::delay(1000);

  drive.move(right, 60, 1, 70);
  pros::delay(1000);

  drive.move(right, 75, 2, 70);
  pros::delay(1000);

  drive.move(right, 85, 2, 70);
  pros::delay(1000);
  
}

void far(){
 imu.set_heading(42);
 wingPis.set_value(true);
 intakePis.set_value(true);
 
 drive.setPID(4);
 drive.move(backward,5,1,70);

 drive.setPID(3);
 drive.move(left, imuTarget(0), 1, 70);

 wingPis.set_value(false);

 drive.setPID(1);
 drive.move(backward, 24, 1, 100);
 moveDriveTrain(-12000, 1);

 drive.move(forward, 14, 1, 100);

 /*
 drive.setPID(3);
 drive.move(left, imuTarget(90), 1, 100);

 wingPis.set_value(true);
 drive.setPID(1);

 //score 1st green
 drive.move(backward, 34, 2, 100);
 intake.move_voltage(0);
 drive.move(forward, 20, 1, 100);

 drive.setPID(2);
 wingPis.set_value(false);

 intake.move_voltage(-4000);
 drive.move(right, imuTarget(270), 1, 100); //180 degree turn 
 
 //score second green 
 drive.setPID(1);
 drive.move(forward, 24, 1, 100); 
 intake.move_voltage(-12000);
 pros::delay(50);
 
 drive.move(right, imuTarget(57), 1, 100);
 intake.move_voltage(12000);
 drive.move(forward, 30, 1, 100);

 */
                 
                  /*{kP,kPt,kI,kIt,kD,kDt,kPd, }*/
 /*
 drive.setCustomPID(0, 64, 0,  2, 0,  6, 0);
 drive.move(left, imuTarget(250), 2, 100); //307 degree turn
 intake.move_voltage(0);

 drive.setPID(2);
 intake.move_voltage(-6000);
 drive.move(forward,34, 1, 100);


 drive.setPID(1);
 drive.move(backward,12, 1, 100);
 */

}

void closeBarTouch(){}

void HighClose(){}
void HighFar(){
 intakePis.set_value(true);
 intake.move_voltage(-12000);
 
 //get first green 
 drive.move(forward, 5, 1, 70);

 drive.move(backward, 32, 1, 100);
 wingPis.set_value(true);
 
 //score proeload and two green
 drive.setPID(7);
 drive.swerve(backwardLeft, 36, imuTarget(270), 2, 70, 100);
 intake.move_voltage(0);

 wingPis.set_value(false);

 drive.setPID(1);
 drive.move(forward, 4, 1, 70);

 drive.setPID(5);
 drive.move(right, imuTarget(90), 2, 70);
 
 intake.move_voltage(12000);
 drive.setPID(4);
 drive.move(forward, 12, 1, 80);

 //get third green
 drive.setPID(3);
 drive.move(backward, 18, 1, 100);
 
 drive.setPID(4);
 drive.move(left, imuTarget(33), 1, 70);

 wingPis.set_value(false);
 
 drive.setPID(3);
 drive.move(forward, 32, 3, 100);

 drive.setCustomPID(0, 260, 0, 0, 0, 173, 0);
 drive.move(right, imuTarget(45), 1, 70);

 intake.move_voltage(-12000);

 drive.setPID(3);
 drive.move(forward, 30, 1, 70);

 //score 4th triball
 drive.move(left, imuTarget(0), 1, 70);
 wingPis.set_value(true);
 
 drive.move(backward, 34, 2, 100);
 intake.move_voltage(0);
 drive.move(forward, 20, 1, 100);

 wingPis.set_value(false);

 //score 5thtriball
 drive.setPID(5);
 drive.move(right, imuTarget(180), 1, 70); 
 
 //get 4th green 6th triball
 intake.move_voltage(-12000);
 drive.setPID(1);
 drive.move(forward, 20, 1, 100);
 
 drive.setPID(4);
 drive.move(backward, 8, 1, 100);

 drive.setPID(2);
 drive.move(right, imuTarget(335), 2, 70);

 drive.setPID(1);
 drive.move(forward, 28, 1, 100);

 drive.move(backward, 20, 1, 100);
 
 //score 6tn triball
 drive.setPID(5);
 drive.move(right,200,1,70);
 
 drive.setPID(1);
 drive.move(forward, 20, 1, 100);
 drive.move(backward, 24, 1, 100);

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