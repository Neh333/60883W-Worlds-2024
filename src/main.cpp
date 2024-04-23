#include "auton.hpp"
#include "drive.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "include.hpp"
#include "lvgl_funcs.hpp"

#define AUTO_NUMBER 6
uint8_t auton = AUTO_NUMBER; 

#define AUTO_SWITCH(){ \
	switch(auton%AUTO_NUMBER){\
    case 0:  controller.print(2, 0, "Close Safe  %.2f                      ",imu.get_heading()); break;\
		case 1:  controller.print(2, 0, "Close Rush Qual %.2f                  ",imu.get_heading()); break;\
		case 2:  controller.print(2, 0, "Far %.2f                              ",imu.get_heading()); break;\
		case 3:  controller.print(2, 0, "Close Rush Elim %.2f                  ",imu.get_heading()); break;\
		case 4:  controller.print(2, 0, "Far Rush %.2f                         ",imu.get_heading()); break;\
		case 5:  controller.print(2, 0, "Nothing %.2f                          ",imu.get_heading()); break;\
	}\
}

void initialize(){
	initBarGraph();
	pros::Task brainDisplayTask(updateBarGraph_fn);
	drive.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  hang.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	imu.reset();
	if(imu.get_heading() || imu.get_rotation() > 0.5){
		imu.reset(); 
	}
} 


void disabled(){
	while(true){
		//Change auton value
		if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		AUTO_SWITCH();
		pros::delay(20);
	}
}


void competition_initialize(){
	while(true){
		//Change auton value
		if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
		if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
		AUTO_SWITCH();
		pros::delay(20);
	}
}


void autonomous(){
 switch(auton%AUTO_NUMBER){
    case 0:  close();         break;
		case 1:  closeRush();	     break;
		case 2:  far();           break;
		case 3:  closeElims();    break;
		case 4:  farRush();       break;
		case 5:  nothing();       break;
    }
}

void set_tank(int l_stick, int r_stick) {
  leftMotors.move_voltage(l_stick * (12000.0 / 127.0));
  rightMotors.move_voltage(r_stick * (12000.0 / 127.0));
}

double left_curve_function(double x, double left_curve_scale) {
  if (left_curve_scale != 0) {
    return (powf(2.718, -(left_curve_scale / 10)) + powf(2.718, (fabs(x) - 127) / 10) * (1 - powf(2.718, -(left_curve_scale / 10)))) * x;
  }
  return x;
}

void arcade_standard(double curve) {

 int fwd_stick, turn_stick;

 // Put the joysticks through the curve function
 fwd_stick = left_curve_function(controller.get_analog(ANALOG_LEFT_Y), curve);
 turn_stick = left_curve_function(controller.get_analog(ANALOG_RIGHT_X), curve);

 // Set robot to l_stick and r_stick, check joystick threshold, set active brake
 set_tank(fwd_stick + turn_stick, fwd_stick - turn_stick);
}

void opcontrol() {
 while (true) {
     //AUTO SELECTOR
     //Display current autonomous on the controller
     AUTO_SWITCH();

     //Change auton value
     if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
     if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
     
      //Run the currently selected autonomous when UP is pressed
     if(controller.get_digital_new_press(DIGITAL_UP)){autonomous();}
     
     //Reset the IMU when the down button is called
     if(controller.get_digital_new_press(DIGITAL_DOWN)){
        imu.reset();
        float iter = 0;
        while(imu.is_calibrating()){
         controller.print(2,0,"Calibrating: %.2f    ", iter/5);
         pros::delay(20);
        }
     }
     
     //DRIVER CONTROL 
     arcade_standard(5);
     
     if (controller.get_digital(DIGITAL_L1)){intake.move_voltage(-12000);}
     else if (controller.get_digital(DIGITAL_L2)) {intake.move_voltage(12000);}
     else {intake.move_velocity(0);}  

     if(controller.get_digital_new_press(DIGITAL_A)){
        fn_Lock = !fn_Lock;
      }
     
     if(fn_Lock){
       if(controller.get_digital_new_press(DIGITAL_R1)){ backWingTog = !backWingTog; }
       if (!backWingTog) {
         backWings.set_value(true);
       }
       else {
        backWings.set_value(false);
       }
     
       if(controller.get_digital_new_press(DIGITAL_R2)){ frontWingTog = !frontWingTog; }
       if (!frontWingTog) {
         frontWings.set_value(true);
       }
       else {
        frontWings.set_value(false);
       }
     } 
 
     else {
       hang.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
       if (controller.get_digital(DIGITAL_R1)){hang.move_voltage(-12000);}
       else if (controller.get_digital(DIGITAL_R2)) {hang.move_voltage(12000);}
       else {hang.move_velocity(0);}   
     }

      if(controller.get_digital_new_press(DIGITAL_Y)){ hangRachet = !hangRachet; }
       if (!hangRachet) {
         hangLock.set_value(true);
       }
       else {
        hangLock.set_value(false);
       }

     pros::delay(20);
    }
}