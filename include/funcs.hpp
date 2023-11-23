#pragma once
#include "include.hpp"

void setBrakeMode(pros::motor_brake_mode_e brakeMode);

void moveLeftDriveTrain(int voltage);
void moveRightDriveTrain(int voltage);
void moveDriveVoltage(int voltage);
void moveDriveVelocity(float velocity);

void moveDriveTrain(int voltage, float time);

int motorAvgLeft();
int motorAvgRight();
int motorAvgAll();

float actualVelocityLeft();
float actualVelocityRight();

//returns a float between -200 and 200
float actualVelocityAll();
//returns a float between 0 and 100, representing how much energy is being used compared to actual movement
float driveEfficiencyAll();

float radToDeg(float radian);
float degToRad(float degree);

int inchToTick(float inch);
float tickToInch(int tick);

float percentToVoltage(float percent);
float voltageToVelocity(float voltage);
int velocityToVoltage(float percent);

float imuTarget(float target);

void puncherDown();

