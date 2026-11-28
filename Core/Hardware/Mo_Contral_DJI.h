//
// Created by Ken on 2026/11/22.
//

#ifndef H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DJI_H
#define H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DJI_H
#include"Overall_Struct.h"

/* 电机控制周期回调：由 My_Tim.c 的 TIM17 中断调用，做 PID 计算 + CAN 发送。
 * 其他 CAN 通道的配置方法见函数内注释。 */
void Mo_Time_Callback_DJI(void);

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

/* ---------------- 精确增量运动（推荐用法） ----------------
 * 底层统一做梯形规划、到位锁定、堵转保护，适合“转 N 圈停住并保持”的场合。
 * 支持全部 DJI 电机类型（GM6020 / C610 / C620），语义相同。
 * 状态：0=空闲，1=运动中，2=已到位保持，3=堵转已卸力 */
typedef enum {
    MO_MOVE_IDLE = 0,
    MO_MOVE_RUN,
    MO_MOVE_HOLD,
    MO_MOVE_STALL
} Mo_MoveState_t;

/* 绝对多圈目标：target_deg 以“上电多圈计数”为基准，可超过 360°
 * 例如 target=24480 表示正转 68 圈，与当前误差无关，天然不累积 */
void Mo_Move_Absolute(uint8_t type, uint8_t id, float target_deg, float v_limit_RPM, uint8_t can_Ch);

/* 相对运动：内部换算成“当前多圈角 + delta”，再走绝对目标 */
void Mo_Move_Relative(uint8_t type, uint8_t id, float delta_deg, float v_limit_RPM, uint8_t can_Ch);

/* 查询运动状态；remain_deg 返回剩余角度（可为 NULL） */
uint8_t Mo_Move_Status(uint8_t type, uint8_t id, float *remain_deg, uint8_t can_Ch);

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
