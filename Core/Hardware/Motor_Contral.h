//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
#define H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
#include "Overall_Struct.h"

extern Mo_PID_Data Mo_GM6020[3][7];//电机编号和电机对应的通道都是0基的
extern Mo_PID_Data Mo_C610[3][7];
extern Mo_PID_Data Mo_C620[3][7];

extern Mo_PID_Data Mo_Demo;
void Motor_Contral_current_Demo(uint16_t current);
void Motor_Callback_Demo(void);
void Motor_Contral_PID(float Speed);

#endif //H7_PREPARED_DEVELOPMENT_MOTOR_CONTRAL_H
