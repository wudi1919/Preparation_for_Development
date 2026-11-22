//
// Created by Ken on 2026/10/18.
//

#include "PID_Calculator.h"
#include "Overall_Struct.h"

PID_Data Mo_PID[Mo_number + 1] = {0};

/**
 * @brief  位置式 PID 计算（速度环单环）
 * @param  p 指向 PIO_Data 的指针（不能为 NULL）
 * @note   内部完成抗饱和积分 + 积分/输出双限幅，结果写回 p->Out
 */

void PID_Calculator(PID_Data * p) {
    if (p == NULL) return;   // 空指针防护，防止死机

    /* 获取本次误差和上次误差 */
    p->Error1 = p->Error0;                       // 上次误差
    p->Error0 = p->Target - p->Actual;           // 本次误差 = 目标 - 实际

    /* 抗饱和积分：输出顶在上限且误差仍为正时停止积分；顶在下限且误差仍为负时停止积分 */
    int sat_hi = (p->Out >=  p->OutMax) && (p->Error0 > 0);
    int sat_lo = (p->Out <= -p->OutMax) && (p->Error0 < 0);
    if (!sat_hi && !sat_lo) {
        p->ErrorInt += p->Error0;
    }

    /* 双保险：对误差积分做一道硬限幅 */
    if (p->ErrorInt >  p->IntMax) p->ErrorInt =  p->IntMax;
    if (p->ErrorInt < -p->IntMax) p->ErrorInt = -p->IntMax;

    /* 位置式 PID 计算 */
    p->Out = p->Kp * p->Error0 + p->Ki * p->ErrorInt + p->Kd * (p->Error0 - p->Error1);

    /* 输出限幅（±OutMax） */
    if (p->Out >  p->OutMax) p->Out =  p->OutMax;
    if (p->Out < -p->OutMax) p->Out = -p->OutMax;
}

/**
 * @brief  配置 PID 参数（Kp/Ki/Kd 及输出、积分限幅）
 * @param  p 指向 PIO_Data 的指针（不能为 NULL）
 */

void PID_config(PID_Data * p, float kp, float ki, float kd, float OutMax, float IntMax) {
    if (p == NULL) return;   // 空指针防护，防止死机
    p->Kp     = kp;
    p->Ki     = ki;
    p->Kd     = kd;
    p->OutMax = OutMax;
    p->IntMax = IntMax;
}
