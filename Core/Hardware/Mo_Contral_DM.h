//
// Created by Ken on 2026/11/26.
//

#ifndef H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DM_H
#define H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DM_H

#include "stm32h7xx_hal.h"
#include "overall_struct.h"
/* 达妙电机编码器类型：1E = 单编码器，2E = 双编码器。
 * 帧结构两类完全相同，差异只在于：位置能否跨圈、是否支持力位混控。 */
typedef enum {
    DM_ENC_1E = 1,   /* 单编码器：位置只有单圈，不能跨圈累计 */
    DM_ENC_2E = 2,   /* 双编码器：电机侧 + 输出轴，输出轴绝对位置可跨圈 */
} DM_Encoder_Type;

typedef enum {
    MTI,
    Speed,
} DM_Work_Mode;

typedef struct  {
    Mo_PID_Data PID_Data;
    DM_Work_Mode Work_mode;
    Mo_DM_Type Mo_Type;
} DM_Mo_Data;


/* 一套完整的达妙 CAN 协议描述。
 * 控制帧 ID = 基地址 + 电机 CAN ID；反馈帧 ID = MasterID；字节序小端。 */
typedef struct {
    const char      *name;          /* 型号标识 */
    DM_Encoder_Type  enc;           /* 编码器类型 */
    uint8_t          multi_turn;    /* 1=输出轴绝对位置可跨圈(2E)，0=单圈(1E) */

    /* 控制帧基地址 */
    uint16_t bar_mit;               /* MIT 混控：p(16b)+v(12b)+Kp(12b)+Kd(12b)+t_ff(12b) */
    uint16_t bar_pos_vel;           /* 位置+速度：D[0..3]=p_des，D[4..7]=v_des（float32） */
    uint16_t bar_vel;               /* 仅速度：D[0..3]=v_des（float32） */
    uint16_t bar_force_pos;         /* 力位混控 */
    uint8_t  has_force_pos;         /* 1=支持力位混控，0=不支持（1E 系列不支持） */

    /* 特殊命令帧：ID = 电机 CAN ID，D[0..6]=0xFF，D[7]=命令字节 */
    uint8_t  cmd_enable;
    uint8_t  cmd_disable;
    uint8_t  cmd_set_zero;
    uint8_t  cmd_clear_err;

    /* 参数读写帧：固定 ID，D[0..1]=CANID(小端)，D[2]=命令，D[3]=寄存器地址 */
    uint16_t param_id;
    uint8_t  cmd_param_read;
    uint8_t  cmd_param_write;
    uint8_t  cmd_param_save;

    /* 反馈帧：ID = MasterID；D[0] = ID | ERR<<4 */
    uint16_t fb_id_default;         /* MasterID 出厂默认值 */
    uint8_t  fb_id_mask;            /* 低 4 位为电机 ID，范围 0~15 */
    uint8_t  fb_err_shift;          /* 高 4 位为错误码 */

    /* MIT 打包位宽：p + v + Kp + Kd + t_ff = 64bit */
    uint8_t  bits_pos;
    uint8_t  bits_vel;
    uint8_t  bits_tor;
    uint8_t  bits_kp;
    uint8_t  bits_kd;

    /* 增益范围（全系列一致） */
    float    kp_min, kp_max;
    float    kd_min, kd_max;

    /* 量程：不同型号/固件不同，0 表示未标定。
     * 使用前按调试助手里读出的实际值填写，否则反馈反算和 MIT 打包都会成比例出错。 */
    float    p_max, v_max, t_max;
} DM_Protocol_t;

/* 1E / 2E 两套协议实例（定义在 Mo_Contral_DM.c） */
extern const DM_Protocol_t DM_Protocol_1E;
extern const DM_Protocol_t DM_Protocol_2E;

/* 按编码器类型索引协议，未知类型按 2E 返回 */
const DM_Protocol_t *DM_Get_Protocol(DM_Encoder_Type enc);

/* 使能电机（指定协议类型）：can_ch=1~3，can_id=电机 CAN ID（0~15） */
void DM_Motor_Enable(uint8_t can_ch, uint16_t can_id, DM_Encoder_Type enc);

/* 使能电机（兼容旧调用，默认按 2E 取协议；1E/2E 的使能命令相同） */
void DM_Enable_Motor(uint8_t can_ch, uint16_t can_id);

DM_Mo_Data Mo_Setting[3][8];
#endif //H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DM_H
