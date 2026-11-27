//
// Created by Ken on 2026/11/26.
//

#ifndef H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DM_H
#define H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DM_H

#include "stm32h7xx_hal.h"
#include "overall_struct.h"

/* 达妙电机编码器类型：1E = 单编码器，2E = 双编码器。
 * 帧结构两类完全相同，差异只在于：位置能否跨圈、是否支持力位混控。 */
void DM_Can_Enabel(uint8_t can_ch1, uint8_t can_ch2, uint8_t can_ch3);
typedef enum {
    DM_ENC_1E = 1,   /* 单编码器：位置只有单圈，不能跨圈累计 */
    DM_ENC_2E = 2,   /* 双编码器：电机侧 + 输出轴，输出轴绝对位置可跨圈 */
} DM_Encoder_Type;

/* 工作模式：当前实现统一走 MIT 帧。
 * MTI = 位置/增量角度控制（位置环 → 速度环 → MIT 力矩接口）
 * Speed = 纯速度闭环（速度环 → MIT 力矩接口） */
typedef enum {
    MTI,
    Speed,
} DM_Work_Mode;

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

/* ==================== 单台电机运行状态 ====================
 * Mo_Setting[3][8] 的索引约定：
 *   [i][j]  i = CAN 通道下标 0~2  →  对应 CAN1 / CAN2 / CAN3
 *           j = 电机下标   0~7
 *               can_id    = j + 1          （电机 CAN ID，1~8）
 *               master_id = can_id + 0x10  （反馈帧 ID，0x11~0x18）
 *   注意：master_id 必须与达妙调试助手里的 Master ID 一致，
 *        出厂常见设置为 CAN ID + 0x10（如 CAN ID=0x01 → Master ID=0x11）。
 *        例：Mo_Setting[0][0] → CAN1、CAN ID=1、反馈 ID=0x11 */
typedef struct  {
    Mo_PID_Data PID_Data;       /* 沿用 DJI 结构：Speed 速度环 + Pos 位置环 + CtrlMode + 重力前馈 */
    DM_Work_Mode Work_mode;
    Mo_DM_Type Mo_Type;

    const DM_Protocol_t *proto; /* 协议表（1E/2E），NULL 表示该槽位未配置 */
    uint16_t can_id;            /* 控制帧 ID = bar + can_id */
    uint16_t master_id;         /* 反馈帧 ID = can_id + 0x10 */

    float p_max;                /* 位置量程 rad（±p_max），反馈反算 + MIT 打包共用 */
    float v_max;                /* 速度量程 rad/s */
    float t_max;                /* 力矩量程 N·m（也是速度环输出限幅） */

    /* MIT 内置位置环增益：0 = 纯力矩接口（PID 全部在 MCU 算）
     * 填非 0 时电机内部再做一层 PD，会与 MCU 的 PID 叠加，一般保持 0。 */
    float kp_mit;
    float kd_mit;

    /* 反馈（均为输出轴量） */
    float pos_rad;              /* 本帧位置 rad，范围 ±p_max */
    float pos_multiturn_rad;    /* 多圈累计位置 rad（已处理回卷） */
    float pos_last_rad;         /* 上帧位置，回卷检测用 */
    float vel_rads;             /* 速度 rad/s */
    float tor_nm;               /* 力矩 N·m */
    uint8_t err;                /* 错误码：0=失能 1=使能 5=传感器 8=过压 9=欠压 A=过流 */
    uint8_t fb_valid;           /* 1 = 收到过有效反馈 */
} DM_Mo_Data;

/* 1E / 2E 两套协议实例（定义在 Mo_Contral_DM.c） */
extern const DM_Protocol_t DM_Protocol_1E;
extern const DM_Protocol_t DM_Protocol_2E;

/* 全部电机状态（定义在 Mo_Contral_DM.c） */
extern DM_Mo_Data Mo_Setting[3][8];

/* 按编码器类型索引协议，未知类型按 2E 返回 */
const DM_Protocol_t *DM_Get_Protocol(DM_Encoder_Type enc);

/* 使能电机（指定协议类型）：can_ch=1~3，can_id=电机 CAN ID（0~15） */
void DM_Motor_Enable(uint8_t can_ch, uint16_t can_id, DM_Encoder_Type enc);

/* 使能电机（兼容旧调用，默认按 2E 取协议；1E/2E 的使能命令相同） */
void DM_Enable_Motor(uint8_t can_ch, uint16_t can_id);

/* ---------------- 配置 ---------------- */

/* 【配置函数】初始化一个电机槽位：填协议、CAN ID、反馈 ID、量程、默认工作模式。
 * can_ch=1~3，idx=0~7（对应 can_id=idx+1、master_id=0x10+can_id）。
 * p_max/v_max/t_max 必须按调试助手里读到的实际值填，否则反馈反算成比例出错。 */
void DM_Mo_Setting_Init(uint8_t can_ch, uint8_t idx, Mo_DM_Type type, DM_Encoder_Type enc,
                        float p_max, float v_max, float t_max);

/* 轨迹参数：v_lim_dps = 斜坡最大速度（度/秒），kp/kd = 电机内部位置环增益。
 * 定角度/增量模式只靠它把目标做成斜坡，位置保持由电机内部 Kp/Kd 完成。 */
void DM_Motion_Config(uint8_t id, float v_lim_dps, float kp, float kd, uint8_t can_ch);

/* 重力前馈：t_ff = k * sin(角度 + phase_deg)，k 单位 N·m */
void DM_Set_Gravity_FF(uint8_t id, float k, float phase_deg, uint8_t can_ch);

/* 修改反馈帧 ID（电机 Master ID 不是 can_id+0x10 时用） */
void DM_Set_MasterID(uint8_t id, uint16_t master_id, uint8_t can_ch);

/* MIT 内置位置环增益（一般保持 0；非 0 会与 MCU 的 PID 叠加） */
void DM_Set_MIT_Gain(uint8_t id, float kp, float kd, uint8_t can_ch);

/* ---------------- 运行时控制（随时可调，只改目标，不发报文） ---------------- */

/* 速度闭环：目标转速 RPM */
void DM_PID_Speed_Control(uint8_t id, int16_t speed_rpm, uint8_t can_ch);

/* 绝对位置控制：转到 0~360° 目标角（取最短路径） */
void DM_PID_Position_Control(uint8_t id, float angle_deg, uint8_t can_ch);

/* 相对增量角度控制：从当前位置再转 delta_deg（可超 360°、多圈） */
void DM_PID_Increment_Control(uint8_t id, float delta_deg, uint8_t can_ch);

/* 直接切换控制模式（DJI_CTRL_POSITION / DJI_CTRL_SPEED / DJI_CTRL_INCREMENT） */
void DM_Set_CtrlMode(uint8_t id, uint8_t mode, uint8_t can_ch);

/* 读取多圈累计角度（度，可超 360） */
float DM_Pos_MultiTurn_Deg(uint8_t id, uint8_t can_ch);

/* ---------------- 周期任务 ---------------- */

/* 【周期回调】由 TIM17 中断调用：遍历所有已配置电机 → PID 计算 → MIT 打包 → 发送。
 * 与 DJI 的 Mo_Time_Callback_DJI() 并列，两者互不影响。
 * 受 Can_DM_Send_Enable() 总开关控制：未使能时整段跳过。 */
void Mo_Time_Callback_DM(void);

#endif //H7_PREPARED_DEVELOPMENT_MO_CONTRAL_DM_H
