//
// Created by Ken on 2026/11/4.
//

#ifndef BLUETOOTH_MACCAR1_0_OVERALL_STRUCT_H
#define BLUETOOTH_MACCAR1_0_OVERALL_STRUCT_H
#include "stm32h7xx.h"

typedef struct {
    float Actual;     // 实际转速 RPM（带符号，可正可负）
    float Target;     // 目标转速 RPM（带符号，可正可负）
    float Error0;     // 本次误差 = Target - Actual（带符号）
    float Error1;     // 上次误差（带符号）
    float ErrorInt;   // 误差积分（带符号）
    float OutMax;     // 输出限幅（±）
    float IntMax;     // 误差积分限幅（±）
    float Kp;
    float Ki;
    float Kd;
    float Out;        // PID 输出（±OutMax，即 ±100 占空比 %）
} PID_Data;

typedef enum {
    GM6020,
    C610,
    C620
}Mo_type;

typedef struct {
    PID_Data Speed;
    PID_Data Angel;
}Mo_PID_Data;


#endif //BLUETOOTH_MACCAR1_0_OVERALL_STRUCT_H
