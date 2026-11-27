//
// Created by Ken on 2026/11/26.
//

#include "HW_Config.h"
#include "Mo_Contral_DJI.h"
#include "Mo_Contral_DM.h"
#include "PID_Calculator.h"
#include "bsp_Can_Callback.h"

/* ==================== 大疆电机配置表 ====================
 * 一行 = 一台电机。加电机就往表里加一行，不需要改任何函数。
 *   can_ch : 1~3 → CAN1~3
 *   id     : 1~8  → C610/C620：1~4 走 0x200、5~8 走 0x1FF；GM6020：1~4 / 5~8
 * 注意：同一个 CAN 通道上的 C610/C620 只能选一种（Mo_Select_DJI 按通道生效）。 */
typedef struct {
    uint8_t     can_ch;
    uint8_t     id;
    Mo_DJI_type type;          /* GM6020 / C610 / C620 */

    float spd_kp, spd_ki, spd_kd;      /* 速度环（位置式 PID） */
    float spd_out_max, spd_int_max;    /* 电流/电压指令限幅、积分限幅 */

    float pos_kp, pos_ki, pos_kd;      /* 位置环（增量式 PID） */
    float pos_rpm_max;                 /* 位置环输出 = 目标转速上限 RPM */

    float grav_k, grav_phase;          /* 重力前馈 k·sin(θ+phase)，k 单位=输出单位 */

    uint8_t mode;                      /* DJI_CTRL_POSITION / SPEED / INCREMENT */
    float   target;                    /* 速度=RPM；位置=角度(0~360°)；增量=增量角(°) */
} DJI_Motor_Cfg_t;

static const DJI_Motor_Cfg_t dji_cfg[] = {
    /* CAN1 / ID5 / C610：速度环 + 位置环，初始转到 90° */
    { .can_ch = 1, .id = 5, .type = C610,
      .spd_kp = 3.0f, .spd_ki = 2.0f, .spd_kd = 0.45f,
      .spd_out_max = 1000.0f, .spd_int_max = 3000.0f,
      .pos_kp = 2.0f, .pos_ki = 0.3f, .pos_kd = 0.0f,
      .pos_rpm_max = 120.0f,
      .grav_k = 0.0f, .grav_phase = 0.0f,
      .mode = DJI_CTRL_POSITION, .target = 90.0f },
};
#define DJI_CFG_NUM (sizeof(dji_cfg) / sizeof(dji_cfg[0]))

/* 按类型取状态结构体指针（越界返回 NULL） */
static Mo_PID_Data *DJI_Stat(Mo_DJI_type type, uint8_t id, uint8_t can_ch)
{
    if (can_ch == 0 || can_ch > 3 || id == 0 || id > 8) return NULL;
    switch (type) {
        case GM6020: return &GM6020_stat[can_ch - 1][id - 1];
        case C610:   return &C610_stat[can_ch - 1][id - 1];
        case C620:   return &C620_stat[can_ch - 1][id - 1];
        default:     return NULL;
    }
}

/* 【配置函数】大疆电机初始化：按表逐台写 PID 参数与初始目标，并按表使能通道 */
void Mo_Init_DJI(void) {
    uint8_t ch_used[3] = {0, 0, 0};

    for (uint32_t i = 0; i < DJI_CFG_NUM; i++) {
        const DJI_Motor_Cfg_t *c = &dji_cfg[i];
        if (c->can_ch == 0 || c->can_ch > 3 || c->id == 0 || c->id > 8) continue;

        /* 该通道的 C610/C620 类型（同一通道只需设一次，重复设同一值无副作用） */
        if (c->type == C610 || c->type == C620) Mo_Select_DJI(c->can_ch, c->type);

        Mo_PID_Data *st = DJI_Stat(c->type, c->id, c->can_ch);
        if (st == NULL) continue;

        /* 速度环（内环） */
        PID_config(&st->Speed, c->spd_kp, c->spd_ki, c->spd_kd, c->spd_out_max, c->spd_int_max);

        /* 位置环（外环）+ 重力前馈 */
        Mo_PID_Position_Config_DJI(c->type, c->id, c->pos_kp, c->pos_ki, c->pos_kd,
                                   c->pos_rpm_max, c->can_ch);
        Mo_Set_Gravity_FF_DJI(c->type, c->id, c->grav_k, c->grav_phase, c->can_ch);

        /* 初始目标 */
        switch (c->mode) {
            case DJI_CTRL_POSITION:
                Mo_PID_Position_Control_DJI(c->type, c->id, c->target, c->can_ch);
                break;
            case DJI_CTRL_INCREMENT:
                Mo_PID_Increment_Control_DJI(c->type, c->id, c->target, c->can_ch);
                break;
            default:
                Mo_PID_Speed_Control_DJI(c->type, c->id, (int16_t)c->target, c->can_ch);
                break;
        }

        ch_used[c->can_ch - 1] = 1;
    }

    /* 表里出现过的通道才开启反馈解析 / PID / 发送 */
    DJI_Can_Enabel(ch_used[0], ch_used[1], ch_used[2]);
}

/* ==================== 达妙电机配置表 ====================
 * 一行 = 一台电机，加电机就往表里加一行。
 *   can_ch : 1~3 → CAN1~3
 *   idx    : 0~7  → CAN ID = idx+1，反馈 ID = 0x11+idx（对应 Mo_Setting[can_ch-1][idx]）
 * 量程必须与调试助手里读到的值一致，否则反馈反算和 MIT 打包会成比例出错。 */
typedef struct {
    uint8_t         can_ch;
    uint8_t         idx;
    Mo_DM_Type      type;      /* 型号标识，仅记录用 */
    DM_Encoder_Type enc;       /* 1E / 2E */

    float p_max, v_max, t_max; /* 位置 rad、速度 rad/s、力矩 N·m（均为 ± 量程） */

    float v_lim_dps;           /* 轨迹斜坡最大速度（度/秒） */
    float mit_kp, mit_kd;      /* 电机内部位置环增益（定角度/增量模式用） */

    float grav_k, grav_phase;  /* 重力前馈 k·sin(θ+phase)，k 单位 N·m */

    uint8_t mode;              /* DJI_CTRL_POSITION / SPEED / INCREMENT */
    float   target;            /* 速度=RPM；位置=角度(0~360°)；增量=增量角(°) */
} DM_Motor_Cfg_t;

static const DM_Motor_Cfg_t dm_cfg[] = {
    /* CAN1 / 槽位0（CAN ID=1，反馈 ID=0x11）/ DM-J4030 / 2E
     * TODO: p_max / v_max / t_max 按调试助手“控制幅值”里读到的实际值改，
     *       填错会导致反馈反算和 MIT 打包整体成比例偏。 */
    { .can_ch = 1, .idx = 0, .type = J4030, .enc = DM_ENC_2E,
      .p_max = 12.5f, .v_max = 20.0f, .t_max = 28.0f,
      .v_lim_dps = 240.0f, .mit_kp = 80.0f, .mit_kd = 3.0f,
      .grav_k = 0.0f, .grav_phase = 0.0f,
      .mode = DJI_CTRL_POSITION, .target = 90.0f },   /* 上电转到 90° */
};
#define DM_CFG_NUM (sizeof(dm_cfg) / sizeof(dm_cfg[0]))

/* 【配置函数】达妙电机初始化：按表逐台配置协议/量程/PID → 使能，并按表使能通道。
 * 控制模式与量程由上位机保存到电机内部，这里只做本地配置 + 使能，不擦写电机 Flash。 */
void Mo_Init_DM(void) {
    uint8_t ch_used[3] = {0, 0, 0};

    for (uint32_t i = 0; i < DM_CFG_NUM; i++) {
        const DM_Motor_Cfg_t *c = &dm_cfg[i];
        if (c->can_ch == 0 || c->can_ch > 3 || c->idx > 7) continue;

        uint8_t id = (uint8_t)(c->idx + 1);   /* 对外接口用 1~8 */

        /* 1. 协议 + CAN ID + 反馈 ID + 量程 */
        DM_Mo_Setting_Init(c->can_ch, c->idx, c->type, c->enc, c->p_max, c->v_max, c->t_max);

        /* 2. 轨迹参数：斜坡速度 + 电机内部位置环 Kp/Kd（定角度/增量模式用） */
        DM_Motion_Config(id, c->v_lim_dps, c->mit_kp, c->mit_kd, c->can_ch);

        /* 3. 重力前馈 */
        DM_Set_Gravity_FF(id, c->grav_k, c->grav_phase, c->can_ch);

        /* 4. 初始目标 */
        switch (c->mode) {
            case DJI_CTRL_POSITION:
                DM_PID_Position_Control(id, c->target, c->can_ch);
                break;
            case DJI_CTRL_INCREMENT:
                DM_PID_Increment_Control(id, c->target, c->can_ch);
                break;
            default:
                DM_PID_Speed_Control(id, (int16_t)c->target, c->can_ch);
                break;
        }

        /* 5. 使能（0xFF×7 + 0xFC），反馈帧 ERR=1 表示成功 */
        DM_Enable_Motor(c->can_ch, id);

        ch_used[c->can_ch - 1] = 1;
    }

    DMI_Can_Enabel(ch_used[0], ch_used[1], ch_used[2]);
}

/* ==================== 统一初始化入口 ==================== */
/* 【配置函数】电机全部初始化，主函数只调一次。
 * 加/换电机只改上面两张表，任何电机相关配置都不要往 main.c 堆。 */
void Mo_Init(void) {
    Mo_Init_DJI();   /* 大疆：按表配置 + 通道使能 */
    Mo_Init_DM();    /* 达妙：按表配置 + 使能 + 通道使能 */
}
