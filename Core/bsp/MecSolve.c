//
// Created by Ken on 2026/11/5.
//

#include "MecSolve.h"
#include "HW_Config.h"

/* omega为正=逆时针ccw，负=顺时针cw。Vx/Vy: m/s，omega: rad/s；输出各轮目标转速 RPM（与 Actual 同单位）\
 * 这里只做底盘的运算所以写死固定的电机
 */

void MecSolve(float Vx, float Vy, float omega, float *out_rpm) {
    out_rpm[1] =  (Vx - Vy - K_chassis * omega) / R_wheel * RAD_S_TO_RPM;   // 左前
    out_rpm[2] = -(Vx + Vy + K_chassis * omega) / R_wheel * RAD_S_TO_RPM;   // 右前
    out_rpm[3] =  (Vx + Vy - K_chassis * omega) / R_wheel * RAD_S_TO_RPM;   // 左后
    out_rpm[4] = -(Vx - Vy + K_chassis * omega) / R_wheel * RAD_S_TO_RPM;   // 右后
}