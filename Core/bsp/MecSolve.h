//
// Created by Ken on 2026/11/5.
//

#ifndef BLUETOOTH_MACCAR1_0_MECSOLVE_H
#define BLUETOOTH_MACCAR1_0_MECSOLVE_H
#include "HW_Config.h"

#define RAD_S_TO_RPM (60.0f / 6.283185307f)   // 1 rad/s = 60/(2π) ≈ 9.549 RPM

/**
 * @brief  麦轮运动学解算：车体速度 → 各轮目标转速
 * @param  Vx      车体前进速度 (m/s，向前为正)
 * @param  Vy      车体横向速度 (m/s，向左为正)
 * @param  omega   车体自转角速度 (rad/s，逆时针为正)
 * @param  out_rpm 输出各轮目标转速 (RPM)，长度 Mo_number+1，下标 1..4 = 左前/右前/左后/右后
 */
void MecSolve(float Vx, float Vy, float omega, float *out_rpm);

#endif //BLUETOOTH_MACCAR1_0_MECSOLVE_H
