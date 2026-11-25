//
// Created by Ken on 2026/11/25.
//

#ifndef H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H
#define H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H

#include <stdint.h>

/* 大疆电机反馈解析：按反馈 ID 计算电机 ID，用枚举区分 GM6020/C610/C620，
 * 把转速/角度写入 Motor_Contral 里对应的 *_stat 结构体 */
void Can_Data_Processing_DJI(uint8_t Can_Ch);

#endif //H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H
