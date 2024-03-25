#pragma once
#include <string>
#define LAMBDA(func) [](){func;}

int signum(double x);
double distBetweenAngles(double targetAngle, double currentAngle);

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


#define LIFT_LOWEST 0
#define LIFT_PLATFORM 1
#define LIFT_OVER_PLATFORM 2
#define LIFT_HIGHEST 3
#define LIFT_RINGS_HEIGHT 210
#define LIFT_RINGS_WITH_MOGO_HEIGHT 225
#define LIFT_OVER_MOGO_HEIGHT 279

//Global Variable Declaration
#define LIFTVALSIZE 3                  /*H, V  , D*/
const float liftTargets[LIFTVALSIZE] = {92, 170, 0}; 

class Hang{
 private:
  //Initialize PID Values
  const float kP = 70;
  const float kI = 0;
  const float kD = 100;
  const float intergralActive = 2;
  const float intergralLimit = 40;

  //Variables that need to be read after each PID scope is destroyed
  float error;
  float lastError;
  float intergral;
    
 public:
  uint8_t targetIndicator = 0;
  float target = liftTargets[2];
  void PID();
  void waitUntilTargetReached(float timeOut);
  void setTarget(int n);
  void setCustomTarget(float target);

};
extern Hang hangPID;
