//
// Created by Ken on 2026/10/19.
//

#ifndef PWM_CHASSIS_CONTROL_HW_CONFIG_H
#define PWM_CHASSIS_CONTROL_HW_CONFIG_H
#include "main.h"

/* ==================== DJI 电机 CAN 标识符（RoboMaster 标准） ====================
 *
 * M3508 减速电机（配 C620 电调）：
 *   电流控制帧：CAN ID = 0x200（一帧发 4 个电机的电流，8 字节 = 4 × int16，单位 mA）
 *               5~8 号电机用 0x1FF（本底盘只用到 4 个）
 *   反馈帧    ：CAN ID = 0x200 + 电机编号（1~8）
 *               每帧 8 字节：[0:1]机械角int16  [2:3]转速int16(RPM)
 *                           [4:5]转矩电流int16(mA)  [6]温度uint8  [7]保留
 *
 * GM6020 云台电机（如需控制云台再补）：
 *   电压控制帧：0x1FF（1~4 号）；反馈帧：0x204 + 编号（1~4 号）
 *
 * 4 个电机编号（麦轮底盘）：
 *   电机 1 = 左前   电机 2 = 右前
 *   电机 3 = 左后   电机 4 = 右后
 * ========================================================================== */

/* 电机数量 */
#define Mo_number 4

/* 电流控制帧（发送给 4 个电调，一次发 4 个电机的电流） */
#define DJI_CURRENT_CTRL_ID    0x200

/* 各电机反馈帧 ID（0x200 + 电机编号） */
#define DJI_MOTOR1_FEEDBACK_ID 0x201   // 电机 1：左前
#define DJI_MOTOR2_FEEDBACK_ID 0x202   // 电机 2：右前
#define DJI_MOTOR3_FEEDBACK_ID 0x203   // 电机 3：左后
#define DJI_MOTOR4_FEEDBACK_ID 0x204   // 电机 4：右后

/* 命令缩放：串口/蓝牙发来的 int16 原始值 / CMD_SCALE = 实际 m/s 或 rad/s */
#define CMD_SCALE 1000.0f

/* 底盘尺寸（麦轮运动学解算用，单位 m） */
#define L_chassis 0.19f
#define W_chassis 0.17f
#define K_chassis ((L_chassis+W_chassis)/2)
#define R_wheel 0.04f

#endif //PWM_CHASSIS_CONTROL_HW_CONFIG_H
/*
 * 这里说明 uart 传输的帧头对应的意思
 * 帧头为 BB 的就是设置 PID 的参数，符号字节 rx_buffer[8]/[9]/[10]：0 为负，1 为正
 * 帧头为 AA 的就是设置 Vx, Vy, omega
 */
