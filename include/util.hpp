#pragma once
#include <string>
#define LAMBDA(func) [](){func;}

int signum(double x);

double inchToTick(double inch);
double tickToInch(double tick);

double percentToVoltage(double percent);

// Max voltage is 12000, max velocity of standard motor is 200 (all that matters is the percent of max speed): 12000/60 = 200
double voltageToVelocity(double voltage);

// 200*60 = 12000
double velocityToVoltage(double velocity);

double imuTarget(double target);

double radToDeg(double rad);

double degToRad(double deg);

struct Triangle {
  public:
   float a;
   float b;
   float hyp;
   float alpha;
   float beta;
};

/**
* Does trigonometry to assign values to attributes of a given Triangle object
*
* @param obj
*        Reference to the Triangle object who's attributes will be set
* @param a
*        Leg A of the triangle
* @param reference_a
*        Angle to subtract from the current IMU heading. 
*        Will set hyp equal to A if reference and IMU heading are the same.   
**/
void findTri(Triangle& obj, double a, double reference_a);