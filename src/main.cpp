#include "auton.hpp"
#include "funcs.hpp"
#include "include.hpp"

#define AUTO_NUMBER 8

//Global Variable Declaration
uint8_t auton = AUTO_NUMBER; 

#define AUTO_SWITCH(){ \
	switch(auton%AUTO_NUMBER){\
		case 0: controller.print(2, 0, "Close  %.2f                       ",imu.get_heading()); break;\
		case 1: controller.print(2, 0, "Far %.2f                          ",imu.get_heading()); break;\
		case 2: controller.print(2, 0, "Close Bar Touch %.2f              ",imu.get_heading()); break;\
		case 3: controller.print(2, 0, "Far High %.2f                     ",imu.get_heading()); break;\
		case 4: controller.print(2, 0, "Close Rush %.2f                   ",imu.get_heading()); break;\
		case 5: controller.print(2, 0, "Far Rush  %.2f                    ",imu.get_heading()); break;\
		case 6: controller.print(2, 0, "Skills %.2f                       ",imu.get_heading()); break;\
		case 7: controller.print(2, 0, "Nothing %.2f                      ",imu.get_heading()); break;\
	}\
}

void initialize(){
	pros::lcd::initialize();
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
		case 0: close();	     break;
		case 1: far();           break;
		case 2: closeBarTouch(); break;
		case 3: HighFar();       break;
		case 4: closeRush();     break;
		case 5: farRush();       break;
		case 6: skills();        break;
		case 7: nothing();       break;
	}
}

void opcontrol(){
 bool wingTog = true;
 bool intakePisTog = false;
 bool liftTog = false;
 //hangPis.set_value(false);
 intakePis.set_value(true);
 setBrakeMode(MOTOR_BRAKE_COAST);
 puncher.set_brake_mode(MOTOR_BRAKE_COAST);
 while (true) {
	 //AUTO SELECTOR
	 //Display current autonomous on the controller
	 AUTO_SWITCH();

	 //Change auton value
	 if(controller.get_digital_new_press(DIGITAL_LEFT)){auton--;}
	 if(controller.get_digital_new_press(DIGITAL_RIGHT)){auton++;}
	 
	  //Run the currently selected autonomous when B is pressed
	 if(controller.get_digital_new_press(DIGITAL_B)){autonomous();}
     
	 //Reset the sensors when the X button is called
	 //Reset the IMU when the up button is called
	 if(controller.get_digital_new_press(DIGITAL_X)){
        imu.reset();
        float iter = 0;
        while(imu.is_calibrating()){
         controller.print(2,0,"Calibrating: %.2f    ", iter/5);
         iter += 20;
         pros::delay(20);
        }
	 }
	 
     //DRIVER CONTROL 
     //create varaibles to hold analog stick values 
     int8_t leftY = controller.get_analog(ANALOG_LEFT_Y);
     int8_t rightX = controller.get_analog(ANALOG_RIGHT_X);
     //Set drive motor speeds
	 int rightDrive = leftY - rightX;
	 frontright = rightDrive;
	 midright = rightDrive;
	 backright = rightDrive;

	 int leftDrive = leftY + rightX;
	 frontleft = leftDrive;
	 midleft = leftDrive;
	 backleft = leftDrive;
     
	 if (controller.get_digital(DIGITAL_L1)){intake.move_velocity(-12000);}
	 else if (controller.get_digital(DIGITAL_L2)) {intake.move_velocity(12000);}
	 else {intake.move_velocity(0);}   

	 if(controller.get_digital_new_press(DIGITAL_A)){ wingTog = !wingTog; }
	 if (!wingTog) {wingPis.set_value(true);}
	 else {wingPis.set_value(false);}

	 if(controller.get_digital_new_press(DIGITAL_Y)){ intakePisTog = !intakePisTog; }
	 if (!intakePisTog) {intakePis.set_value(false);}
	 else {intakePis.set_value(true);}

	 if(controller.get_digital_new_press(DIGITAL_UP)){ liftTog = !liftTog; }
	 if (!liftTog) {hangPis.set_value(false);}
	 else {hangPis.set_value(true);}
	 
	 if (controller.get_digital(DIGITAL_R1)) { puncher.move_voltage(10000);}
	 else if (controller.get_digital(DIGITAL_R2)) {puncher.move_voltage(12000);}
	 else {puncher.move_voltage(0);}
	}
} 