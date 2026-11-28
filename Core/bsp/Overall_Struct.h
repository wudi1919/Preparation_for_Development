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
    float Out;        // PID 输出（±OutMax）
    uint8_t Saturated; // 输出饱和标志：1 = 本次输出被限幅截断（供堵转检测用）
} PID_Data;

/* 增量式 PID（位置环用）：输出为“增量累加”，天然抗积分饱和、目标跳变不突变。
 * 标准离散形式：Δu = Kp*(e-e1) + Ki*e + Kd*(e-2e1+e2)，dt 已折入 Ki/Kd。
 * 位置环固定 20ms 周期，增益在此周期下整定。 */
typedef struct {
    float Error0;     // e[k]   本次误差
    float Error1;     // e[k-1] 上次误差
    float Error2;     // e[k-2] 上上次误差
    float Out;        // 累加输出（= 目标转速 RPM）
    float OutMax;     // 输出限幅（±，目标转速上限）
    float Target;     // 目标参考（达妙轨迹斜坡用：最终目标角）
    float Kp;
    float Ki;
    float Kd;
} IncPID_Data;

typedef enum {
    GM6020,
    C610,
    C620
}Mo_DJI_type;

/* 控制模式：位置保持 / 速度环 / 相对增量角度 */
typedef enum {
    DJI_CTRL_POSITION  = 0,   // 绝对位置控制（默认，0~360° 取最短路径）
    DJI_CTRL_SPEED     = 1,   // 速度环（现有行为）
    DJI_CTRL_INCREMENT = 2    // 相对增量角度控制（可超 360°，多圈累计）
} DJI_CtrlMode;

typedef struct {
    PID_Data    Speed;          // 速度环（位置式 PID）→ 力矩/电压
    IncPID_Data Pos;            // 位置环（增量式 PID）→ 目标转速
    float       LastTurnDeg;    // 上次测速采样的累计角度，做差分测速（度）
    uint8_t     CtrlMode;       // DJI_CtrlMode
    float       PosTargetDeg;   // 位置目标：绝对模式=目标角(0~360)；增量模式=增量角度
    float       IncStartDeg;    // 增量模式起点累计角度（度）
    float       GravityK;       // 重力前馈增益（输出单位）
    float       GravityPhase;   // 重力前馈相位（度）
} Mo_PID_Data;

typedef enum {
    S3519,
    J4340,
    J4340P,
    J6216P,
    J6216,
    J4030,
}Mo_DM_Type;
#endif //BLUETOOTH_MACCAR1_0_OVERALL_STRUCT_H
