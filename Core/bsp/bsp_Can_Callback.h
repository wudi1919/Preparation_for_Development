//
// Created by Ken on 2026/11/25.
//

#ifndef H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H
#define H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H

#include <stdint.h>

/* 大疆电机反馈解析：按反馈 ID 计算电机 ID，用枚举区分 GM6020/C610/C620，
 * 把转速/角度写入 Motor_Contral 里对应的 *_stat 结构体 */
void Can_Data_Processing_DJI(uint8_t Can_Ch);

/* T 法测速：读取 C610 角度差分累积值并清零，按调用周期换算成 RPM。
 * 必须在固定周期（TIM17, 20ms）调用。低速精度远高于电调直报速度。 */
float Motor_DiffSpeed_RPM_C610(uint8_t can_ch, uint8_t id);

#endif //H7_PREPARED_DEVELOPMENT_BSP_CAN_CALLBACK_H
