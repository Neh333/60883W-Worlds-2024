#include "funcs.hpp"
#include "include.hpp"
#include "pros/rtos.hpp"


void setBrakeMode(pros::motor_brake_mode_e brakeMode) {
  frontright.set_brake_mode(brakeMode);
  midright.set_brake_mode(brakeMode);
  backright.set_brake_mode(brakeMode);
  frontleft.set_brake_mode(brakeMode);
  midleft.set_brake_mode(brakeMode);
  backleft.set_brake_mode(brakeMode);
}

void moveLeftDriveTrain(int voltage) {
  frontright.move_voltage(voltage);
  midright.move_voltage(voltage);
  backright.move_voltage(voltage);
}

void moveRightDriveTrain(int voltage) {
  frontleft.move_voltage(voltage);
  midleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveVoltage(int voltage) {
  frontright.move_voltage(voltage);
  midright.move_voltage(voltage);
  backright.move_voltage(voltage);
  frontleft.move_voltage(voltage);
  midleft.move_voltage(voltage);
  backleft.move_voltage(voltage);
}

void moveDriveTrain(int voltage, float time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
  moveDriveVoltage(0);
}


int motorAvgLeft() {
  return (backleft.get_position() + midleft.get_position() + frontleft.get_position()) / 3;
}

int motorAvgRight() {
  return (frontright.get_position() + midright.get_position() + backright.get_position()) / 3;
}

int motorAvgAll() {
  return (motorAvgLeft() + motorAvgRight()) / 2;
}

float actualVelocityLeft() {
  return (frontleft.get_actual_velocity() + midleft.get_actual_velocity() + backleft.get_actual_velocity()) / 3;
}

float actualVelocityRight() {
  return (frontright.get_actual_velocity() + midright.get_actual_velocity() + backright.get_actual_velocity()) / 3;
}

// returns a float between -200 and 200
float actualVelocityAll() {
  return (actualVelocityLeft() + actualVelocityRight()) / 2;
}

// returns a float between 0 and 100, representing how much energy is being used compared to actual movement
float driveEfficiencyAll() {
  return ((frontright.get_efficiency() + midright.get_efficiency() + backright.get_efficiency() +
           frontleft.get_efficiency() + midleft.get_efficiency() + backleft.get_efficiency()) /6);
}

float radToDeg(float radian){return (radian * (180 / Pi));}

float degToRad(float degree){return (degree * (Pi / 180));}

/*
 *1800 ticks/rev with 100 rpm cartridge
 *900 ticks/rev with 200 rpm cartridge
 *300 ticks/rev with 600 rpm cartridge

 *450 ticks for one full wheel rotation (300 * (72/48) or (3/2)) (reversed gear ratio)
 *Circumference of 4" omni = 12.9590696961 (4.125*pi)
 *450 ticks / 12.9590696961 = 34.7247148563 ticks per inch
 */
int inchToTick(float inch) {
  return (inch * 34.7247148563);
}

float tickToInch(int tick) {
  return (tick / 34.7247148563);
}

float percentToVoltage(float percent) {
  return (percent * 120);
}

// Max voltage is 12000, max velocity of standard motor is 200 (all that matters is the percent of max speed): 12000/60 = 200
float voltageToVelocity(float voltage) {
  return (voltage / 60);
}

// 200*60 = 12000
int velocityToVoltage(float velocity) {
  return (velocity * 60);
}

/**
 * Find the shortest distance between two angles
 * \param target
 *        Desired angle
 * \returns Shortest rotation between current and target angle in degrees
 *
 */

/*
float imuTarget(float target){
  return fabs(fmod((target-imu.get_heading()+360),360));
}
*/
float imuTarget(float target){
  return(fabs(fmod((target-imu.get_heading()+360),360)));
}

void puncherDown(){
   puncherR.set_data_rate(5);
   float pPos = puncher.get_position()/100;
   float target = 5;

   const uint32_t cutOff = pros::millis() + 1000;
   
   if (pPos <= target) {puncher.move_voltage(0);}
   else{puncher.move_voltage(4500);}
   
   while ((pros::millis() < cutOff) && (pPos >= target)) {
    //Wait until the cata is where it should be
    pros::delay(20);
   }
   puncher.move_voltage(0);
}
