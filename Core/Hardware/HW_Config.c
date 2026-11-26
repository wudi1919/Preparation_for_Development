//
// Created by Ken on 2026/11/26.
//

#include "HW_Config.h"
#include "Mo_Contral_DJI.h"
#include "Mo_Contral_DM.h"
#include "PID_Calculator.h"

/* 【配置函数】电机初始化 + PID 参数配置，全部放这里，主函数只调一次。
 * 底层驱动（Mo_Contral_DJI.c）只提供通用接口，具体选型/参数/目标在这里配置。 */
void Mo_Init_DJI(void) {
    /* 选择 CAN1 上 C610/C620 类型为 C610 */
    Mo_Select_DJI(1, C610);

    /* 速度环（内环，位置式 PID）：C610 电流指令 ±1000（约 ±1A） */
    PID_config(&C610_stat[0][4].Speed, 3.0f, 2.0f, 0.45f, 1000.0f, 3000.0f);

    /* 位置环（外环，增量式 PID）：输出目标转速，OutMax=±120 RPM */
    Mo_PID_Position_Config_DJI(C610, 5, 2.0f, 0.3f, 0.0f, 120.0f, 1);

    /* 重力前馈：C610 无重力负载，k=0（GM6020 机械臂需标定 k/phase） */
    Mo_Set_Gravity_FF_DJI(C610, 5, 0.0f, 0.0f, 1);

    /* 初始目标：转到 90°（位置环模式，TIM17 中断每 20ms 自动保持到位） */
    Mo_PID_Position_Control_DJI(C610, 5, 90.0f, 1);
}

/* ==================== 达妙电机配置表 ====================
 * 达妙全系列帧格式一致，只有 CAN 通道 / 电机 CAN ID 需要按实车填写。
 * 新增电机：往表里加一行即可。反馈帧 ID 走默认 MasterID=0（DM_MASTER_ID_DEFAULT）。 */
typedef struct {
    uint8_t  can_ch;   /* 1~3 → hfdcan1~3 */
    uint16_t can_id;   /* 电机 CAN ID，范围 0~15 */
} DM_Motor_Cfg_t;

static const DM_Motor_Cfg_t dm_cfg[] = {
    /* TODO: 按实际接线修改 CAN 通道与电机 ID */
    { 1, 0x01 },
};
#define DM_CFG_NUM (sizeof(dm_cfg) / sizeof(dm_cfg[0]))

/* 【配置函数】达妙电机初始化：逐个使能，主函数只调一次。
 * 控制模式（MIT / 位置速度 / 速度 / 力位混控）、P_MAX/V_MAX/T_MAX 量程
 * 由上位机配好并保存到电机内部，这里不切换，避免反复擦写电机 Flash。 */
void Mo_Init_DM(void) {
    for (uint32_t i = 0; i < DM_CFG_NUM; i++) {
        DM_Enable_Motor(dm_cfg[i].can_ch, dm_cfg[i].can_id);
        HAL_Delay(2);
    }
}
