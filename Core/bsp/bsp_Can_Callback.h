//
// Created by Ken on 2026/11/25.
//

#ifndef H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H
#define H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H

#include <stdint.h>

/* ==================== 通道级使能 ====================
 * 参数分别对应 CAN1 / CAN2 / CAN3，1=开启，0=关闭。
 * 开启：该通道的反馈解析 + PID 计算 + CAN 报文发送都生效；
 * 关闭：三者全部跳过，省算力也省总线。默认全部关闭。 */
void DJI_Can_Enabel(uint8_t can_ch1, uint8_t can_ch2, uint8_t can_ch3);
void DMI_Can_Enabel(uint8_t can_ch1, uint8_t can_ch2, uint8_t can_ch3);

/* 查询某通道是否已开启（can_ch = 1~3） */
uint8_t DJI_Can_IsEnabled(uint8_t can_ch);
uint8_t DM_Can_IsEnabled(uint8_t can_ch);

/* 大疆电机反馈解析：按反馈 ID 计算电机 ID，用枚举区分 GM6020/C610/C620，
 * 把转速/角度写入 Mo_Contral_DJI 里对应的 *_stat 结构体 */
void Can_Data_Processing_DJI(uint8_t Can_Ch);

/* 达妙电机反馈解析：按 MasterID 匹配 Mo_Setting 槽位，
 * 解出位置/速度/力矩，并做 ±p_max 回卷累计成多圈角度 */
void Can_Data_Processing_DM(uint8_t Can_Ch);

/* 读取某电机多圈累计角度（度，可超 360）。由 CAN 回调累积回卷，
 * 供 Mo_Contral_DJI 差分测速 + 位置环使用。 */
float DJI_Angle_MultiTurn_Deg(uint8_t can_ch, uint8_t id);

#endif //H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H
