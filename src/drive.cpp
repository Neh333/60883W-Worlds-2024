#include "drive.hpp"

#include "util.hpp"
#include <cmath>
#include <math.h>
#include "include.hpp"
#include <algorithm>

/* onError function to be executed as a task */
void onError_fn(void* param)
{
  pros::Mutex onErrorMutex;
  std::uint32_t startTime = pros::millis();
  while (true){
    onErrorMutex.take();
    if(drive.getPIDStatus())
    {
      auto iter = drive.onErrorVector.begin();
      while(iter != drive.onErrorVector.end())
      {
        if (!iter->called && (iter->onError >= drive.getError()))
        {
          (iter->func)();
          iter->called = true;
        }
        else iter++;
      }
    }
    onErrorMutex.give();
    pros::Task::delay_until(&startTime, 10);  
  }
}


/* Add an error-function tuple to the onErrorVector */
void Drive::addErrorFunc(double onError, void input())
{
  onErrorVector.emplace_back(errorFuncTuple(input, inchToTick(onError), false));
}

// Set PID constants
/*Set PID constants //changes the row of the array used by taking n and subtracting it by one so set 1 would be row 0 of the array
*Ex: setPID(2) would be set 2 which is located at array[1](the second row of the array)
*Array found in odom.hpp named "pidConstants"
*/
/*Set PID constants */
void Drive::setPID(uint8_t n)
{
  n--;
  kP   = PIDConstants[n].kP;
  kP_a = PIDConstants[n].kP_a;
  kI   = PIDConstants[n].kI;
  kI_a = PIDConstants[n].kI_a;
  kD   = PIDConstants[n].kD;
  kD_a = PIDConstants[n].kD_a;
  kP_d = PIDConstants[n].kP_d;
}


/* Use specified PID constants */
void Drive::setCustomPID(PIDprofile profile)
{
  kP   = profile.kP;
  kP_a = profile.kP_a;
  kI   = profile.kI; 
  kI_a = profile.kI_a;
  kD   = profile.kD;
  kD_a = profile.kD_a;
  kP_d = profile.kP_d;
}


void Drive::setScheduledConstants(PIDprofile constants)
{
  scheduledConstants = constants;
}


void Drive::setScheduleThreshold_l(double error)
{
  scheduleThreshold_l = error;
}


void Drive::setScheduleThreshold_a(double error)
{
  scheduleThreshold_a = error;
}

void Drive::setMaxVelocity(float velocity) {
 this->maxVolt = percentToVoltage(velocity);
}

void Drive::setMaxTurnVelocity(float velocity) {
 this->maxVolt_a = percentToVoltage(velocity);
}

/* Update slewProf given profile */
void Drive::setSlew(slewProfile profile)
{
  slewProf = profile;
}

/* Update slewProf given profile */
void Drive::setSlew_a(slewProfile profile)
{
  slewProf_a = profile;
}


void Drive::setStandStill(movement_Type type, uint8_t maxCycles, float maxStep) {
 if (type == lateral) {
     // Deactivate standstill if maxcycles is 0
     SSActive = !(maxCycles == 0);
     SSMaxCount = maxCycles;
     maxStepDistance = maxStep;
    } 

 else if (type == turn_type) {
     // Deactivate standstill if maxcycles is 0
     SSActive_t = !(maxCycles == 0);
     SSMaxCount_t = maxCycles;
     maxStepTurn = maxStep;
 }

}

/*********************************************************************************************************/
//Updater methods 

/* Updates voltage given the slewProfile */
void Drive::calculateSlew(double *voltage, double actualVelocity, slewProfile *profile){
{
  if(profile->slew && (fabs(actualVelocity) > profile->slew_lower_thresh) && (fabs(actualVelocity) < profile->slew_upper_thresh))
  {
    double deltaVelocity = voltageToVelocity(*voltage) - actualVelocity;
    if(fabs(deltaVelocity) > profile->slew)
    {
      *voltage = velocityToVoltage(actualVelocity + (profile->slew * signum(deltaVelocity)));
    }
  }
}
}

void Drive::updateIntegral(double error, double lastError, double activeDistance, double &integral) {
  // Reset integral when crossing the zero point (going from error being positive to negative of vice versa)
  if ((error < 0) != (lastError < 0)) {
    integral = 0;
  }
  // Otherwise only continue to accumulate if within the active range
  else if (fabs(error) <= activeDistance) {
     integral += error;
     integral = std::clamp(integral,-INTEGRAL_MAX,INTEGRAL_MAX);
   }  else {
     integral = 0;
   }
}

 void Drive::updateStandstill(movement_Type type, bool &standStill, double error, double lastError, uint8_t &standStillCount) {
   if (type == lateral) {
     if (SSActive && fabs(lastError - error) <= maxStepDistance) {
       standStillCount++;
       if (standStillCount > SSMaxCount) {
         standStill = true;
       }
     } else {
       standStillCount = 0;
     }
   } 
  
   else if (type == turn_type) {
     if (SSActive_t && fabs(lastError - error) <= maxStepTurn) {
       standStillCount++;
       if (standStillCount > SSMaxCount_t) {
         standStill = true;
       }
     } else {
       standStillCount = 0;
      }
    }
}

 
/* Returns the result of the PID calculation, updates integral */
double Drive::updatePID(double KP, double KI, double KD, double error, double lastError, 
                      double &integral, double integralActive)
{
  updateIntegral(error, lastError, integralActive, integral);
  
  const double derivative = error - lastError;

  return KP*error + KI*integral + KD*derivative;
}

double Drive::updatePD(double KP, double KD, double error, double lastError)
{
  const double derivative = error - lastError;

  return KP*error + KD*derivative;
}
  
  

/*********************************************************************************************************/
//Movement Functions
 
/* Basic linear PID movement function */
double Drive::move(Direction dir, double target, double timeOut, double maxVelocity){
  /* Error values */
  double lastError;
  double errorDrift;
  double proportionDrift;
  const double initialHeading = imu->get_heading();
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_l == NO_SCHEDULING);
  double myKP = kP, myKI = kI, myKD = kD;
  /* Integral declaration */
  double integral = 0;
  /* Motor output variable declarations */
  maxVolt = percentToVoltage(maxVelocity);
  double finalVolt;
  /* Drive output multiplier */
  const int8_t reverseVal = (dir == backward)?(-1):(1);
  /* Standstill variable declarations */
  uint8_t standStillCount = 0;
  bool standStill = false;
  /* Tell the onError task that a new PID has begun, and set endTime */
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(pros::millis() < endTime && !standStill)
  {
        /* Maybe schedule constants */
    if(!scheduled && fabs(error) < scheduleThreshold_l)
    {
      myKP = scheduledConstants.kP;
      myKI = scheduledConstants.kI;
      myKD = scheduledConstants.kD;
      scheduled = true;
    }

    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    finalVolt = updatePID(myKP, myKI, myKD, error, lastError, integral, integralActive);
    calculateSlew(&finalVolt, actualVelocityAll(), &slewProf);
    finalVolt = std::clamp(finalVolt, -maxVolt, maxVolt);

    // Print statement used for testing
    controller.print(2, 0, "Error: %.2f", tickToInch(error));

    // Check if robot is in standstill and updates the standstill flag accordingly
    updateStandstill(lateral, standStill, error, lastError, standStillCount);

    /* Update lastError */
    lastError = error;
  
    /* Calculate the product of heading drift and kP_d */
    errorDrift = fmod((initialHeading-(imu->get_heading())+540),360) - 180;
    proportionDrift = errorDrift * kP_d;

    /* Move Drivetrain */
    moveRightDriveVoltage((reverseVal*finalVolt)+proportionDrift);
    moveLeftDriveVoltage((reverseVal*finalVolt)-proportionDrift);
    
    /* Give PROS time to keep itself in order */
    pros::delay(20);
  }

  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  isNewPID = false;
  return tickToInch(error);
}


/* Basic angular PID movement function */
double Drive::turn(Direction dir, double target, double timeOut, double maxVelocity)
{ 
  double lastError;
  const double initialAngle = imu->get_rotation() + 360;
  /* Scheduling variables */
  bool scheduled = (scheduleThreshold_a == NO_SCHEDULING);
  double myKP = this->kP_a, myKI = this->kI_a, myKD = this->kD_a;
  /* Integral definition */
  double integral = 0;
  /* Motor output variable declarations */
  maxVolt_a = percentToVoltage(maxVelocity);
  double finalVolt;
  /* Drive output multiplier */
  int8_t reverseVal = (dir == right)?(1):(-1);
  /* Standstill variable declarations */
  uint8_t standStillCount = 0;
  bool standStill = false;

  /* Change the reverseVal and target if the direction input is shortest */
  if(dir == shortest)
  {
    target = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal = signum(target);
    target = fabs(target);
  }
  
  /* Tell the onError task that a new PID has begun, and set endTime */
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while((pros::millis() < endTime && !standStill)){
    if(!scheduled && fabs(error) < scheduleThreshold_a)
    {
      myKP = scheduledConstants.kP_a;
      myKI = scheduledConstants.kI_a;
      myKD = scheduledConstants.kD_a;
      scheduled = true;
    }

    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error = target - fabs(imu->get_rotation() + 360 - initialAngle);
    finalVolt = updatePID(myKP, myKI, myKD, error, lastError, integral, integralActive_a);
    calculateSlew(&finalVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
    finalVolt = std::clamp(finalVolt, -maxVolt_a, maxVolt_a);

    // Print statement used for testing
    controller.print(2, 0, "Error: %.2f", error);

    /* Calculate standstill */
    updateStandstill(lateral, standStill, error, lastError, standStillCount);
    
    /* Update lastError */
    lastError = error;

    /* Move Drivetrain */
    moveRightDriveVoltage(reverseVal*-finalVolt);
    moveLeftDriveVoltage(reverseVal*finalVolt);
    
    /* Give PROS time to keep itself in order */
    pros::delay(20);
  }

  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  isNewPID = false;
  return error;
}


 
double Drive::hardStop(Direction dir, double targetCutOff, double target, double maxVelocity){
   double errorDrift;
   double proportionDrift;
   double lastError;
   const double initialHeading = imu->get_heading();
   const double initialMotorAvg = driveAvgPos();
   const double tickTarget = inchToTick(target);
   const double tickTargetCutOff = inchToTick(targetCutOff);
   /* Scheduling variables */
   bool scheduled = (scheduleThreshold_l == NO_SCHEDULING);
   double myKP = kP, myKI = kI, myKD = kD;
   //Motor output var declarations//
   maxVolt = percentToVoltage(maxVelocity);
   double finalVolt;
 
   //Forward Backward movement multipliers
   const int8_t reverseVal = dir == backward ? 1: -1;
   //Tell the onError task that a new PID has begun
   isNewPID = true;

   //Begin PID
   while(tickTargetCutOff > fabs(driveAvgPos()-initialMotorAvg)){
        /* Maybe schedule constants */
     if(!scheduled && fabs(error) < scheduleThreshold_l){
       myKP = scheduledConstants.kP;
       myKI = scheduledConstants.kI;
       myKD = scheduledConstants.kD;
       scheduled = true;
     }
     error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    
     //update the PD values
     updatePD(kP, kD, error, lastError);
     
     //print error value for tuning
     controller.print(2,0, "Error: %.2f", tickToInch(error));
     
     /* Calculate the product of heading drift and kP_d */
     errorDrift = fmod((initialHeading-imu->get_heading()+540),360) - 180;
     proportionDrift = errorDrift * kP_d;

     //Set Drivetrain
     moveRightDriveVoltage((reverseVal*finalVolt)+proportionDrift);
     moveLeftDriveVoltage((reverseVal*finalVolt)-proportionDrift);

     //Give PROS time to keep itself in order
     pros::delay(20);
    }
    //Set voltage to 0 in case this is the last function called in an autonomous
    moveDriveVoltage(0);
    //Tell the onError task that the PID is over
    isNewPID = false;
    //Exit the function
    return tickToInch(error);
}

//TO DO: potentailly add scheduling 
/* Swerve movement */
double Drive::swerve(Direction dir, double target, double target_a, double timeOut, double maxVel, double maxVel_a){
  /* Error values */
  double error_a;
  double lastError;
  double lastError_a;
  const double initialMotorAvg = driveAvgPos();
  const double tickTarget = inchToTick(target);
  const double initialAngle = imu->get_rotation() + 360;
  /* Integral declarations */
  double integral = 0;
  double integral_a = 0;
  /* Motor output variable declarations */
  maxVolt = percentToVoltage(maxVel);
  maxVolt_a = percentToVoltage(maxVel_a);;
  double workingVolt;
  double finalVoltLeft;
  double finalVoltRight;
  /* Drive output multipliers */
  const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight || dir == backwardShortest)?(-1):(1);
  int8_t reverseVal_a = (dir == backwardRight || dir == forwardRight)?(1):(-1);

  /* Change the reverseVal and target if the direction input is shortest */
  if(dir == forwardShortest || dir == backwardShortest)
  {
    target_a = fabs(fmod((target-imu->get_heading()+540),360) - 180);
    reverseVal_a = signum(target_a);
    target_a = fabs(target_a);
  }

  /* Standstill variable declarations */
  uint8_t standStillCount = 0;
  uint8_t standStillCount_a = 0;
  bool standStill = false;
  bool standStill_a = false;

  /* Tell the onError task that a new PID has begun, and set endTime */
  isNewPID = true;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  /* Begin PID */
  while(pros::millis() < endTime && !(standStill && standStill_a))
  {
    /********************************************DRIVE***************************************************/
    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
    workingVolt = updatePID(kP, kI, kD, error, lastError, integral, integralActive);
    calculateSlew(&workingVolt, actualVelocityAll(), &slewProf);
    workingVolt = std::clamp(workingVolt, -maxVolt, maxVolt);

    /* Calculate standstill */
    updateStandstill(lateral, standStill, error, lastError, standStillCount);
    
    /* Update lastError */
    lastError = error;
  
    finalVoltRight = reverseVal*workingVolt;
    finalVoltLeft = reverseVal*workingVolt;
    /********************************************TURN****************************************************/
    /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
    error_a = target_a - fabs(imu->get_rotation() + 360 - initialAngle);
    workingVolt = updatePID(kP_a, kI_a, kD_a, error_a, lastError_a, integral_a, integralActive_a);
    calculateSlew(&workingVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
    workingVolt = std::clamp(workingVolt, -maxVolt_a, maxVolt_a);

    //print error value for tuning
    controller.print(2,0, "Error: %.2f", tickToInch(error));

    /* Calculate standstill */
    updateStandstill(lateral, standStill_a, error_a, lastError_a, standStillCount_a);
    
    /* Update lastError */
    lastError_a = error_a;
    
    finalVoltRight += reverseVal_a*(-workingVolt);
    finalVoltLeft += reverseVal_a*(workingVolt);
    /********************************************MOVE****************************************************/
    //master.print(2,0,"%.2f ,%.2f, %.2f            ", tickToInch(error), error_a, target_a);

    /* Move Drivetrain */
    moveRightDriveVoltage(finalVoltRight);
    moveLeftDriveVoltage(finalVoltLeft);

    /* Give PROS time to keep itself in order */
    pros::delay(20);
  }
  
  /* Tell the onError task that the PID is over, then return the error at time of exit */
  moveDriveVoltage(0);
  isNewPID = false;
  return tickToInch(error);
}

double Drive::hardStopSwerve(Direction dir, double targetCutOff, double target, double target_a,
                              double maxVel, double maxVel_a){
 /* Error values */
 double error_a;
 double lastError;  
 double lastError_a;

 const double initialHeading = imu->get_heading();
 const double initialMotorAvg = driveAvgPos();
 const double tickTarget = inchToTick(target);
 const double tickTargetCutOff = inchToTick(targetCutOff);
 const double initialAngle = imu->get_rotation() + 360;

 /* Integral declarations */
 double integral = 0;
 double integral_a = 0;
 /* Motor output variable declarations */
 maxVolt = percentToVoltage(maxVel);
 maxVolt_a = percentToVoltage(maxVel_a);;
 double workingVolt;
 double finalVoltLeft;
 double finalVoltRight;

 /* Drive output multipliers */
 const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight || dir == backwardShortest)?(-1):(1);
 int8_t reverseVal_a = (dir == backwardRight || dir == forwardRight)?(1):(-1);
 const bool isShortest = (dir == forwardShortest || dir == backwardShortest)?(true):(false);

 /* Change the reverseVal and target if the direction input is shortest */
 if(dir == forwardShortest || dir == backwardShortest)
  {
   target_a = fabs(fmod((target-imu->get_heading()+540),360) - 180);
   reverseVal_a = signum(target_a);
   target_a = fabs(target_a);
  }
 
 // Standstill variables//
 uint8_t standStillCount, standStillCount_a = 0;
 bool standStill, standStill_a = false;
 // Tell the onError task that a new PID has begun
 isNewPID = true;

 // Begin PID
 while (tickTargetCutOff > fabs(driveAvgPos()-initialMotorAvg)) {
     /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
     error = tickTarget - fabs(driveAvgPos() - initialMotorAvg);
     workingVolt = updatePD(kP, kD, error, lastError);
     calculateSlew(&workingVolt, actualVelocityAll(), &slewProf);
     workingVolt = std::clamp(workingVolt, -maxVolt, maxVolt);
    
     /* Update lastError */
     lastError = error;
  
     finalVoltRight = reverseVal*workingVolt;
     finalVoltLeft = reverseVal*workingVolt;

     /********************************************TURN****************************************************/
     /* Update error, do actual PID calculations, adjust for slew, and clamp the resulting value */
     error_a = target_a - fabs(imu->get_rotation() + 360 - initialAngle);
     workingVolt = updatePD(kP_a, kD_a, error_a, lastError_a);
     calculateSlew(&workingVolt, actualVelocityLeft() - actualVelocityRight(), &slewProf_a);
     workingVolt = std::clamp(workingVolt, -maxVolt_a, maxVolt_a);

     finalVoltRight += reverseVal_a*(-workingVolt);
     finalVoltLeft += reverseVal_a*(workingVolt);

     //controller.print(2,0,"%.2f ,%.2f ", tickToInch(error), error_a);
     //controller.print(0, 0, "error_a: %.2f", error_a);

     // Set drivetrain voltages
     moveRightDriveVoltage(finalVoltRight);
     moveLeftDriveVoltage(finalVoltLeft);

     // Give the brain time to keep itself in order
     pros::delay(20);
    }
   //Set voltage to 0 in case this is the last function called in an autonomous
   moveDriveVoltage(0);
   //Tell the onError task that the PID is over
   isNewPID = false;
   //Exit the function
   return tickToInch(error);
  }

  
/* Actively halt robot movement for timeOut seconds */
double Drive::brake(double timeOut)
{
  const double target = driveAvgPos();
  const double kP_brake = 30;
  const uint32_t endTime = pros::millis() + timeOut*1000;

  while(pros::millis() < endTime)
  {
      error = target - driveAvgPos();
      moveDriveVoltage(error*kP_brake);
      pros::delay(20);
  }

  return tickToInch(error);
}


/*********************************************************************************************************/
//Getters 
const double Drive::getError(){
   return this->error;
}

const bool Drive::getPIDStatus(){
   return this->isNewPID;
}

/*********************************************************************************************************/
//Drive methods 
double Drive::leftDriveAvgPos(){
  double value = 0;
  for (int i = 0; i<(leftMotors->size()); i++) {
    value += this->leftMotors->get_positions().at(i);
  }
  return value/leftMotors->size();
}

double Drive::rightDriveAvgPos(){
   double value = 0;
   for (int i = 0; i<(rightMotors->size()); i++) {
    value += this->rightMotors->get_positions().at(i);
  }
  return value/rightMotors->size();
}

double Drive::driveAvgPos(){
 return (leftDriveAvgPos()+rightDriveAvgPos())/2;
}

double Drive::actualVelocityLeft(){
 double value = 0;
 for (int i = 0; i<(leftMotors->size()-1); i++) {
    value += this->leftMotors->get_actual_velocities().at(i);
 }
 return value/leftMotors->size();
}

double Drive::actualVelocityRight(){
   double value = 0;
   for (int i = 0; i<rightMotors->size(); i++) {
    value += this->rightMotors->get_actual_velocities().at(i);
  }
  return value/rightMotors->size();
}

double Drive::actualVelocityAll(){
   return (actualVelocityLeft()+actualVelocityRight())/2;
}

void Drive::moveLeftDriveVoltage(int voltage) {
  leftMotors->move_voltage(voltage);
}

void Drive::moveRightDriveVoltage(int voltage) {
  rightMotors->move_voltage(voltage);
}

void Drive::moveDriveVoltage(int voltage) {
  moveLeftDriveVoltage(voltage);
  moveRightDriveVoltage(voltage);
}

void Drive::moveDriveTrain(int voltage, float time){
  moveDriveVoltage(voltage);
  pros::delay(time*1000);
  moveDriveVoltage(0);
}

void Drive::setBrakeMode(pros::motor_brake_mode_e brakeMode) {
  leftMotors->set_brake_modes(brakeMode);
  rightMotors->set_brake_modes(brakeMode);
}
