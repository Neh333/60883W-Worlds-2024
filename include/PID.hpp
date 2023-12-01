#include "funcs.hpp"
#define integralMax 41.0f

// Direction enumeration
enum PID_dir {
  forward,
  backward,
  left,
  right,
  forwardRight,
  forwardLeft,
  backwardRight,
  backwardLeft,
};

// Flags used for standstill calculations
enum SS_flags {
  lateral,
  turn
};

class Drive {
  private:
  // Initialize PID Values
  const float pidConstants[6][7] = {
  /*{kP, kPt, kI, kIt, kD,  kDt, kPd}*/
    {34, 118, 0, 82,  100, 555, 185}, //90 - 240 / gen lat
    {21, 185, 8, 51,  80,  732, 275}, //30 - 85 / short lat

    {54, 70,  0, 0,   245, 160,   0},  // 1st swerve of high far 

    {0,   0,  0,  0,    0,  0, 0},  //aggresive swerves (far side)
    {0,   0,  0,  0,    0,  0, 0},  //? swerves
    {0,   0,  0,  0,    0,  0, 0},  //goal swerves (skills)
  };

  float kP;
  float kP_t;
  float kI;
  float kI_t;
  float kD;
  float kD_t;
  float kP_d;

  float slew = 0;
  float slewMin = 30;
  float slewMax = 60;

  float error;
  float maxVolt;
  float maxVolt_a;

  float maxStepDistance = 2;
  float maxStepTurn = 0.2;

  float SSMaxCount = 10;
  float SSMaxCount_t = 10;

  bool SSActive = true;
  bool SSActive_t = true;

  bool isNewPID = false;

  float headingValue();
  float rotationValue();

  float calculateSlew(float actualVelocity, float finalVolt);
  void updateIntegral(float error, float lastError, float activeDistance, float &integral);
  void updateStandstill(SS_flags dir, bool &standStill, float error, float lastError, uint8_t &standStillCount);

  public:
  // Functions ran when Drive is initialized
  Drive() {
   setPID(1);
  }

  ~Drive() {}

  // Setters
  void setPID(uint8_t n);
  void setMaxVelocity(float velocity);
  void setMaxTurnVelocity(float velocity);
  void setCustomPID(float kp, float kp_t, float ki, float ki_t, float kd, float kd_t, float kp_d);
  void setSlew(float slew, float lowerThresh, float upperThresh);
  void setStandStill(SS_flags dir, uint8_t maxCycles, float largestStep);
  void addErrorFunc(float onError, void input());

  // Getters
  float getError();
  bool getPIDStatus();

  // Movement Functions, Return error after movement is finished
  float move(PID_dir dir, float target, float timeOut, float maxVelocity);
  float swerve(PID_dir dir, float target, float target_a, float timeOut, float maxVel, float maxVel_a);
};