//
// Created by Ken on 2026/10/18.
//

#ifndef PWM_CHASSIS_CONTROL_CALCULATOR_H
#define PWM_CHASSIS_CONTROL_CALCULATOR_H
#include "Overall_Struct.h"
#include "HW_Config.h"

void PID_Calculator(PID_Data * p);
void PID_config(PID_Data * p, float kp, float ki, float kd, float OutMax, float IntMax);
/* 清空运行状态（积分/误差历史/输出），保留 Kp/Ki/Kd 与限幅；切控制模式时调用 */
void PID_Reset(PID_Data * p);
/* 设置积分分离带宽（0 = 使用默认 200 RPM）；一般取该环最大目标值的 1.2 倍 */
void PID_Set_IBand(PID_Data * p, float band);

/* 增量式 PID：位置环用。target/actual 单位一致（度），Out 输出为目标转速（RPM） */
void PID_Incremental_Calc(IncPID_Data * p, float target, float actual);
void PID_Incremental_Config(IncPID_Data * p, float kp, float ki, float kd, float OutMax);



#endif //PWM_CHASSIS_CONTROL_CALCULATOR_H
