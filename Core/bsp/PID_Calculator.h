//
// Created by Ken on 2026/10/18.
//

#ifndef PWM_CHASSIS_CONTROL_CALCULATOR_H
#define PWM_CHASSIS_CONTROL_CALCULATOR_H
#include "Overall_Struct.h"
#include "HW_Config.h"

void PID_Calculator(PID_Data * p);
void PID_config(PID_Data * p, float kp, float ki, float kd, float OutMax, float IntMax);



#endif //PWM_CHASSIS_CONTROL_CALCULATOR_H
