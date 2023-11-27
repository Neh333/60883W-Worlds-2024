#include "funcs.hpp"
#include "PID.hpp"
#include "include.hpp"
#include <algorithm>
#include <bits/stdc++.h>
#include <vector>

float Drive::headingValue() {
  return imu.get_heading();
}

float Drive::rotationValue() {
  return imu.get_rotation();
}

float Drive::calculateSlew(float actualVelocity, float targetVoltage) {
  // Actual velocity should be a number between +-200, to

  float voltageDifference = targetVoltage - velocityToVoltage(actualVelocity);

  /*Min function checks if the slew value is less than  voltage diffrence and if it is assigns change to slew
  * else set abs val of VD to change
  */
  float change = MIN(percentToVoltage(slew), fabs(voltageDifference));

  return std::clamp(actualVelocity + (voltageDifference >= 0 ? change : -change), -maxVolt, maxVolt);
}

void Drive::updateIntegral(float error, float lastError, float activeDistance, float &integral) {
  // Reset integral when crossing the zero point (going from error being positive to negative of vice versa)
  if ((error < 0) != (lastError < 0)) {
    integral = 0;
  }
  // Otherwise only continue to accumulate if within the active range
  else if (fabs(error) <= activeDistance) {
    integral += error;
    integral = std::clamp(integral,-integralMax,integralMax);
  } else {
    integral = 0;
  }
}


void Drive::updateStandstill(SS_flags dir, bool &standStill, float error, float lastError, uint8_t &standStillCount) {
  if (dir == lateral) {
    if (SSActive && fabs(lastError - error) <= maxStepDistance) {
      standStillCount++;
      if (standStillCount > SSMaxCount) {
        standStill = true;
      }
    } else {
      standStillCount = 0;
    }
  } else if (dir == turn) {
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

// Set PID constants
/*Set PID constants //changes the row of the array used by taking n and subtracting it by one so set 1 would be row 0 of the array
*Ex: setPID(2) would be set 2 which is located at array[1](the second row of the array)
*Array found in odom.hpp named "pidConstants"
*/
void Drive::setPID(uint8_t n) {
  n--;
  kP = pidConstants[n][0];
  kP_t = pidConstants[n][1];
  kI = pidConstants[n][2];
  kI_t = pidConstants[n][3];
  kD = pidConstants[n][4];
  kD_t = pidConstants[n][5];
  kP_d = pidConstants[n][6];
}

void Drive::setCustomPID(float kp, float kp_t, float ki, float ki_t, float kd, float kd_t, float kp_d){
  kP = kp;
  kP_t = kp_t;
  kI = ki;
  kI_t = ki_t;
  kD = kd;
  kD_t = kd_t;
  kP_d = kp_d;
}

void Drive::setMaxVelocity(float velocity) {
  maxVolt = percentToVoltage(velocity);
}

void Drive::setMaxTurnVelocity(float velocity) {
  maxVolt_a = percentToVoltage(velocity);
}

void Drive::setSlew(float n, float lowerThresh, float upperThresh) {
  slew = n;
  slewMin = lowerThresh;
  slewMax = upperThresh;
}

void Drive::setStandStill(SS_flags dir, uint8_t maxCycles, float maxStep) {
  if (dir == lateral) {
    // Deactivate standstill if maxcycles is 0
    SSActive = !(maxCycles == 0);
    SSMaxCount = maxCycles;
    maxStepDistance = maxStep;
  } else if (dir == turn) {
    // Deactivate standstill if maxcycles is 0
    SSActive_t = !(maxCycles == 0);
    SSMaxCount_t = maxCycles;
    maxStepTurn = maxStep;
  }
}

float Drive::getError(){
  return error;
}

bool Drive::getPIDStatus(){
  return isNewPID;
}

/********************************************************************************************************/

// Basic PID movement function
float Drive::move(PID_dir dir, float target, float timeOut, float maxVelocity) {
  // Error values
  float lastError;
  const float initialMotorAvg = motorAvgAll();
  // Calc var declarations
  float &proportion = error;
  float integral;
  float derivative;
  // Motor output var declarations
  maxVolt = percentToVoltage(maxVelocity);
  float finalVolt;
  // Forward Backward movement multipliers
  int8_t reverseVal = (dir == backward || dir == right) ? (-1) : (1);
  // Standstill variable declerations
  uint8_t standStillCount = 0;
  bool standStill = false;
  // Establish cutoff time
  const uint32_t endTime = pros::millis() + timeOut * 1000;

  if (dir == forward || dir == backward) {
    // Drive specific variables
    const float tickTarget = inchToTick(target);
    const float initialAngle_d = headingValue();
    const float integralActive = inchToTick(1);
    float errorDrift;
    float proportionDrift;

    // Begin the PID loop
    while (pros::millis() < endTime && !standStill) {

      // Calculate error (used directly for proportion)
      error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);

      // Calculate and update the integral term
      updateIntegral(error, lastError, integralActive, integral);

      // Calculate the derivative term
      derivative = error - lastError;

      // Check if robot is in standstill and updates the standstill flag accordingly
      updateStandstill(lateral, standStill, error, lastError, standStillCount);

      // Calculate the value that will be fed into max voltage and slew
      finalVolt = std::clamp(kP * proportion + kI * integral + kD * derivative, -maxVolt, maxVolt);

      // Print statement used for testing
      controller.print(2, 0, "Error: %.2f", tickToInch(error));

      // Adjust for slew if enabled and needed
      if (slew && (fabs(actualVelocityAll()) > slewMin) && (fabs(actualVelocityAll()) < slewMax)) {
        finalVolt = calculateSlew(actualVelocityAll(), finalVolt);
      }

      // Counteract rotational drift
      errorDrift = fmod((initialAngle_d - headingValue() + 540), 360) - 180;
      proportionDrift = errorDrift * kP_d;

      // Update last error for next cycle
      lastError = error;

      // Set Drivetrain voltages
      moveRightDriveTrain((reverseVal * finalVolt) + proportionDrift);
      moveLeftDriveTrain((reverseVal * finalVolt) - proportionDrift);

      // Give the brain time to keep itself in order
      pros::delay(20);
    }
    // Stop the robot for 20 milliseconds
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveDriveVoltage(0);
    pros::delay(20);
    setBrakeMode(MOTOR_BRAKE_COAST);
    // Update onError flag to reflect that the PID is over
    isNewPID = false;
    // Exit the function
    return tickToInch(error);
  }

  /********************************************TURN****************************************************/

  else if (dir == right || dir == left) {
    // Turn specific variables
    const float initialAngle = rotationValue() + 360;
    const float integralActive_t = 4;

    // Begin PID
    while ((pros::millis() < endTime && !standStill)) {

      // Calculate error (used directly for proportion)
      error = target - fabs(rotationValue() + 360 - initialAngle);

      // Calculate and update the integral term
      updateIntegral(error, lastError, integralActive_t, integral);

      // Calculate the derivative term
      derivative = error - lastError;

      // Check if robot is in standstill and updates the standstill flag accordingly
      updateStandstill(turn, standStill, error, lastError, standStillCount);

      // Calculate the value that will be fed into max voltage and slew
      finalVolt = std::clamp((kP_t * proportion) + (kI_t * integral) + (kD_t * derivative), -maxVolt, maxVolt);

      // Print statement used for testing
      controller.print(2, 0, "Error: %.2f", error);

      // Adjust for slew if enabled and needed
      // TODO TEST TO ENSURE THAT SLEW WORKS FOR TURNS
      float turnSpeed = (actualVelocityLeft() - actualVelocityRight()) / 2;
      if (slew && (fabs(turnSpeed) > slewMin) && (fabs(turnSpeed) < slewMax)) {
        finalVolt = calculateSlew(turnSpeed, finalVolt);
      }

      lastError = error;

      // Set drivetrain voltages
      moveRightDriveTrain(reverseVal * -finalVolt);
      moveLeftDriveTrain(reverseVal * finalVolt);

      // Give the brain time to keep itself in order
      pros::delay(20);
    }
    // Stop the robot for 20 milliseconds
    setBrakeMode(MOTOR_BRAKE_HOLD);
    moveDriveVoltage(0);
    pros::delay(20);
    setBrakeMode(MOTOR_BRAKE_COAST);
    // Update onError flag to reflect that the PID is over
    isNewPID = false;
    // Exit the function
    return error;
  }
  return 0;
}

/********************************************************************************************************/

float Drive::swerve(PID_dir dir, float target, float target_a, float timeOut, float maxVel, float maxVel_a) {
  float finalValueLeft;
  float finalValueRight;
  float error_a;
  float lastError;
  float lastError_a;
  const float initialAngle = rotationValue() + 360;
  const float initialMotorAvg = motorAvgAll();
  const float tickTarget = inchToTick(target);
  // Calc var declarations//
  float &proportion = error;
  float &proportion_a = error_a;
  float integral;
  float integral_a;
  float derivative;
  float derivative_a;
  // integral var declarations//
  const float integralActive = inchToTick(3);
  const float integralActive_a = 2.2;
  // Motor output var declarations//
  maxVolt = percentToVoltage(maxVel);
  maxVolt_a = percentToVoltage(maxVel_a);
  float finalVolt;
  // Take shortest directions into account
  // Forward Backward movement multipliers//
  const int8_t reverseVal = (dir == backwardLeft || dir == backwardRight) ? (-1) : (1);
  const int8_t reverseVal_a = (dir == backwardRight || dir == forwardRight) ? (-1) : (1);
  // Standstill variables//
  uint8_t standStillCount, standStillCount_a = 0;
  bool standStill, standStill_a = false;
  // Tell the onError task that a new PID has begun//
  isNewPID = true;
  // End time variable declaration//
  const uint32_t endTime = pros::millis() + timeOut * 1000;
  // Begin PID//
  while (pros::millis() < endTime && !(standStill && standStill_a)) {
    /********************************************DRIVE***************************************************/

    // Calculate current error which is used for proportion
    error = tickTarget - fabs(motorAvgAll() - initialMotorAvg);

    // Calculate and update the integral term
    updateIntegral(error, lastError, integralActive, integral);

    // Update the derivative term
    derivative = error - lastError;

    // Check if robot is in standstill and updates the standstill flag accordingly
    updateStandstill(lateral, standStill, error, lastError, standStillCount);

    // Calculate the voltage of the drive portion of the swerve
    finalVolt = std::clamp(kP * proportion + kI * integral + kD * derivative, -maxVolt, maxVolt);

    // Calculate slew if enabled and needed
    if (slew && (fabs(actualVelocityAll()) > slewMin) && (fabs(actualVelocityAll()) < slewMax)) {
      finalVolt = calculateSlew(actualVelocityAll(), finalVolt);
    }
    //Print statement used for testing
    controller.print(2, 0, "Error: %.2f", tickToInch(error));

    lastError = error;

    finalValueRight = finalVolt * reverseVal;
    finalValueLeft = finalVolt * reverseVal;

    /********************************************TURN****************************************************/

    // Update error differently depending on if a "shortest" direction is being used
    error_a = target_a - fabs(rotationValue() + 360 - initialAngle);

    // Calculate and update the integral term
    updateIntegral(error_a, lastError_a, integralActive_a, integral_a);

    // Update the derivative term
    derivative_a = error_a - lastError_a;

    // TODO POTENTIALLY REMOVE STANDSTILL CHECK FOR ANGULAR COMPONENT OF SWERVE
    // Check if robot is in standstill and updates the standstill flag accordingly
    updateStandstill(turn, standStill_a, error_a, lastError, standStillCount_a);

    // Calculate the voltage of the turn portion of the swerve
    finalVolt = std::clamp(kP_t * proportion_a + kI_t * integral_a + kD_t * derivative_a, -maxVolt_a, maxVolt_a);

    finalValueRight -= finalVolt * reverseVal_a;
    finalValueLeft += finalVolt * reverseVal_a;

    lastError_a = error_a;

    // Set drivetrain voltages
    moveRightDriveTrain(finalValueRight);
    moveLeftDriveTrain(finalValueLeft);

    // Give the brain time to keep itself in order
    pros::delay(20);
  }
  // Stop the robot for 20 milliseconds
  setBrakeMode(MOTOR_BRAKE_HOLD);
  moveDriveVoltage(0);
  pros::delay(20);
  setBrakeMode(MOTOR_BRAKE_COAST);
  // Tell the onError task that the PID is over
  isNewPID = false;
  // Exit the function
  return tickToInch(error);
}