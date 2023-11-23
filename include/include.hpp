#pragma once
#include "main.h"
#define MIN(a,b) ((a)<(b)?(a):(b)) //takes param "A" & "B" if A is less than B then A if not then B 
#define MAX(a,b) ((a)>(b)?(a):(b)) //takes param "A" & "B" if A is greather than than B then A if not then B 
#define Pi 3.1415926535 //Macro for using Pi 

//declare controller 
extern pros::Controller controller;

//Declare motors
extern pros::Motor frontright;
extern pros::Motor midright;
extern pros::Motor backright;

extern pros::Motor frontleft;
extern pros::Motor midleft;
extern pros::Motor backleft;

extern pros::Motor puncher;
extern pros::Motor intake;

//Declare V5 sensors
extern pros::Rotation puncherR;
extern pros::Imu imu;
extern pros::Vision vison;

//pistions
extern pros::ADIDigitalOut intakePis;
extern pros::ADIDigitalOut wingPis;
extern pros::ADIDigitalOut hangPis;
