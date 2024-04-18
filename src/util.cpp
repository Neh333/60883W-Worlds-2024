#include "util.hpp"
#include <algorithm>
#include <cmath>
#include "include.hpp"
#include "pros/rtos.hpp"

/* Returns 1 if x is positive or zero or -1 if x is negative */
int signum(double x)
{
  return (x >= 0) - (x < 0);
}


/* Find the shortest distance between current angle and a desired angle */
double imuTarget(double target)
{
  return(fabs(fmod((target-imu.get_heading()+540),360) - 180));
}

/*
*1800 ticks/rev with 100 rpm cartridge
*900 ticks/rev with 200 rpm cartridge
*300 ticks/rev with 600 rpm cartridge

*400 ticks for one full wheel rotation (300 * (48/36) or (4/3)) (reversed gear ratio)
*Circumference of 3.25" omni = 10.2101761242 (3.25*pi)
*400 ticks / 10.2101761242  = 39.1766013763 ticks per inch
*/
double inchToTick(double inch) {
   return (inch * 39.1766013763);
}

double tickToInch(double tick) {
   return (tick / 39.1766013763);
}

double percentToVoltage(double percent) {
  return (percent * 120);
}

// Max voltage is 12000, max velocity of standard motor is 200 (all that matters is the percent of max speed): 12000/60 = 200
double voltageToVelocity(double voltage) {
  return (voltage / 60);
}

// 200*60 = 12000
double velocityToVoltage(double velocity) {
   return (velocity * 60);
}

void findTri(Triangle& obj, double a, double reference_a){
  obj.beta = degToRad(imu.get_heading() - reference_a); 
  obj.alpha = 90 - obj.beta;
  obj.a = a;
  obj.b =  a * tan(obj.beta);
  obj.hyp = sqrt(a*a + obj.b*obj.b);
}