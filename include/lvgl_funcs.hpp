#pragma once
#include "include.hpp"
#define BAR_OFFSET 35
#define BAR_HEIGHT 200
#define BAR_WIDTH 30
#define BAR_RANGE_MIN 0
#define BAR_RANGE_MAX 100
#define ANIM_TIME 200
constexpr float VAL_LABEL_CONSTANT = BAR_HEIGHT/(BAR_RANGE_MAX - BAR_RANGE_MIN);

#define WARNING_THRESH 45
#define DANGER_THRESH 55


//Define the names of the motors
#define BAR_MOTOR_1 frontleft
#define BAR_MOTOR_2 midleft
#define BAR_MOTOR_3 backleft
#define BAR_MOTOR_4 frontright
#define BAR_MOTOR_5 midright
#define BAR_MOTOR_6 backright
#define BAR_MOTOR_7 intake
#define BAR_MOTOR_8 puncher


void setBarAttributes(lv_obj_t* barObj, lv_obj_t* allignTo, int offSet = BAR_OFFSET);
void setBarValAndColor(lv_obj_t* barObj, int val, lv_obj_t* labelObj);
void initBarGraph();
void updateBarGraph();
void updateBarGraph_fn(void* param);