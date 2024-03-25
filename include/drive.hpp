#pragma once
#include "main.h"
#include "util.hpp"


#define INTEGRAL_MAX 200.0
#define NO_SCHEDULING -1.f

enum Direction{
  forward,
  backward,
  left,
  right,
  shortest,
  forwardRight,
  forwardLeft,
  forwardShortest,
  backwardRight,
  backwardLeft,
  backwardShortest
};

// Flags used for standstill calculations
enum movement_Type {
  lateral,
  turn_type
};

struct errorFuncTuple
{
  std::function<void()> func;
  double onError;
  bool called;

  errorFuncTuple(std::function<void()> func, double onError, bool called): func(func), onError(onError), called(called){}
};

struct PIDprofile
{
  double kP;
  double kP_a;
  double kI;
  double kI_a;
  double kD;
  double kD_a;
  double kP_d;
  double kP_s;
};

struct slewProfile
{
  double slew;
  double slew_lower_thresh;
  double slew_upper_thresh;
};

// Initialize PID Values
 const PIDprofile PIDConstants[6] = {
 /*{kP, kPt,  kI, kIt, kD,  kDt,  kPd}*/
   {0,  130,   0,   0,  0,  200,    0}, //

   {0,  40,   0,   0,  0,  570,    0}, //

   {49, 215,  1,   90, 250, 670,    0}, //
 
   {31,  30,  3,   0,  140,  160,   0}, //

   {64, 148,  3,   0,  240,  240,   0}, //

   {26, 175,  0,   0,  220,  160,   0}, //
};

class Drive{
 private:

  double kP, kP_a, kI, kI_a, kD, kD_a, kP_d;

  PIDprofile scheduledConstants;
  double scheduleThreshold_l, scheduleThreshold_a; 
  
  double error;
  
  const double integralActive = inchToTick(3);
  const int integralActive_a = 3;

  double maxVolt;
  double maxVolt_a;
  
  // Standstill variable declerations
  double maxStepDistance = 2;
  double maxStepTurn = 0.2;

  double SSMaxCount = 7;
  double SSMaxCount_t = 7;

  bool SSActive = true;
  bool SSActive_t = true;
  
  //on error flag
  bool isNewPID = false;
  
  //PID updater methods 
  double updatePID(double KP, double KI, double KD, double error, double lastError, double &integral, 
                 double integralActive);
  double updatePD(double KP, double KI, double KD, double error, double lastError);
  void updateIntegral(double error, double lastError, double activeDistance, double& integral);
  void updateStandstill(movement_Type type, bool& standStill, double error, double lastError,
                         uint8_t& standStillCount);

  void calculateSlew(double *voltage, double actualVelocity, slewProfile *profile);

  //"Virtual" Drivetrain methods
  double leftDriveAvgPos(); 
  double rightDriveAvgPos();
  double driveAvgPos();
   
  // returns a float between -200 and 200
  double actualVelocityLeft();
  double actualVelocityRight();
  double actualVelocityAll();

  // returns a float between 0 and 100, representing how much energy is being used compared to actual movement
  double driveEfficiencyAll();

  struct slewProfile slewProf;
  struct slewProfile slewProf_a;

  public:
  //Drive object constructor 
  Drive(pros::MotorGroup &leftMotors, pros::MotorGroup &rightMotors, pros::Imu &imu){
    setPID(1);
    setScheduleThreshold_l(NO_SCHEDULING);
    setScheduleThreshold_a(NO_SCHEDULING);

    /* slew, slew_lower_thresh, slew_upper_thresh */
    setSlew({0, 30, 80});
    setSlew_a({0, 30, 80});

    this->leftMotors    = &leftMotors;
    this->rightMotors   = &rightMotors;
    this->imu           = &imu;
  }

  //"Virtual" Drivetrain attributes and methods 
  pros::MotorGroup *rightMotors;
  pros::MotorGroup *leftMotors;
  pros::Imu        *imu;

  void setBrakeMode(pros::motor_brake_mode_e brakeMode);

  void moveLeftDriveVoltage(int voltage);
  void moveRightDriveVoltage(int voltage);
  void moveDriveVoltage(int voltage); 

  void moveDriveTrain(int voltage, float time);

  //setters
  void setMaxVelocity(float velocity);
  void setMaxTurnVelocity(float velocity);
  void setCustomPID(PIDprofile profile);

  void setScheduledConstants(PIDprofile constants);
  void setScheduleThreshold_l(double error);
  void setScheduleThreshold_a(double error);

  void setSlew(slewProfile profile);
  void setSlew_a(slewProfile profile);
  void setStandStill(movement_Type type, uint8_t maxCycles, float largestStep);
  void setPID(uint8_t n);
  
  // Getters
  const double getError();
  const bool getPIDStatus();

  //On Error
  /* The onError vector */
  std::vector<errorFuncTuple> onErrorVector;
  void addErrorFunc(double onError, void input());

  /* Movement Functions, Return error after movement is finished */
  double move(Direction dir, double target, double timeOut, double maxVelocity);
  double turn(Direction dir, double target, double timeOut, double maxVelocity);

  //Hardstop fn for PID motion chaining 
  double hardStop(Direction dir, double targetCutOff, double target, double maxVelocity);
  
  //Swerve Movemnet Function                 
  double swerve(Direction dir, double target, double target_a, double timeOut, double maxVel, 
                 double maxVel_a);
  
  //Hardstop swerve fn for PID motion chaining 
  double hardStopSwerve(Direction dir, double targetCutOff, double target, double target_a,
                         double maxVel, double maxVel_a);
};

extern Drive drive;

