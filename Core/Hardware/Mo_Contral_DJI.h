//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DJI_H
#define H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DJI_H
#include"Overall_Struct.h"

/* 【配置函数】电机初始化 + PID 参数配置，统一放这里，主函数只调一次 */
void Mo_Init_DJI(void);

/* 【配置函数】启动 TIM17 电机控制定时器（20ms 周期回调做 PID + CAN 发送）。
 * 其他 CAN 通道的配置方法见函数内注释。 */
void Mo_Tim_Init_DJI(void);

/* 设置某路 CAN 上 C610/C620 的类型（每个通道只会是 C610 或 C620 其中一种） */
void Mo_Select_DJI(uint8_t Can_Ch, uint8_t type);

/* 设置某个电机的电流指令，写入对应的 Tx 缓冲 */
void Mo_Set_current_DJI(uint8_t type, uint8_t id, int16_t current, uint8_t can_Ch);

/* 把指定 CAN 通道的 Tx 缓冲按标识符发送出去 */
void Mo_Callback_DJI(uint8_t Can_Ch, uint8_t GM6020_Ch, uint8_t C620_And_C610_Ch);

/* 速度环 PID：目标转速 → PID 计算 → 电流写入 Tx 缓冲（会切到速度环模式） */
void Mo_PID_Speed_Control_DJI(uint8_t type, uint8_t id, int16_t Speed, uint8_t can_Ch);

/* 设置控制模式：DJI_CTRL_POSITION / DJI_CTRL_SPEED / DJI_CTRL_INCREMENT */
void Mo_Set_CtrlMode_DJI(uint8_t type, uint8_t id, uint8_t mode, uint8_t can_Ch);

/* 绝对位置控制：转到 0~360° 目标角（取最短路径），进入位置环模式 */
void Mo_PID_Position_Control_DJI(uint8_t type, uint8_t id, float angle_deg, uint8_t can_Ch);

/* 相对增量角度控制：以当前位置为起点转 delta_deg（度，可超 360），进入增量模式 */
void Mo_PID_Increment_Control_DJI(uint8_t type, uint8_t id, float delta_deg, uint8_t can_Ch);

/* 配置位置环（增量式 PID），OutMax 为目标转速上限（RPM） */
void Mo_PID_Position_Config_DJI(uint8_t type, uint8_t id, float kp, float ki, float kd, float out_max, uint8_t can_Ch);

/* 配置重力前馈：F = k * sin(角度 + phase_deg)（加在力矩上，度） */
void Mo_Set_Gravity_FF_DJI(uint8_t type, uint8_t id, float k, float phase_deg, uint8_t can_Ch);

/* 周期回调：跑速度环 PID 并发送 */
void Mo_PID_Callbakc_DJI(uint8_t Can_Ch, uint8_t GM6020_Ch, uint8_t C620_And_C610_Ch);

/* 反馈状态数组（由 Can_Data_Processing_DJI 写入转速/角度） */
extern Mo_PID_Data GM6020_stat[3][8];
extern Mo_PID_Data C620_stat[3][8];
extern Mo_PID_Data C610_stat[3][8];
extern uint8_t CAN3__C620_and_C610_Type_select[3];

#endif //H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DJI_H
