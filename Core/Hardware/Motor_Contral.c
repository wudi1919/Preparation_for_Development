//
// Created by Ken on 2026/11/22.
//

#include "Motor_Contral.h"
#include"HW_Config.h"
#include"Overall_Struct.h"

uint8_t CAN1_GM6020[8];//考虑到can总线负载每一路只考虑使用最多8个大疆的电机所以id是1到4还是5到六也需要单独配置
uint8_t CAN1_C620_and_C610[8];

uint8_t CAN2_GM6020[8];//考虑到can总线负载每一路只考虑使用最多8个大疆的电机所以id是1到4还是5到六也需要单独配置
uint8_t CAN2_C620_and_C610[8];

uint8_t CAN3_GM6020[8];//考虑到can总线负载每一路只考虑使用最多8个大疆的电机所以id是1到4还是5到六也需要单独配置
uint8_t CAN3_C620_and_C610[8];


Mo_PID_Data GM6020_stat[3][8];
Mo_PID_Data C620_stat[3][8];
Mo_PID_Data C610_stat[3][8];
uint8_t CAN3__C620_and_C610_Type_select[3];

void Mo_Select(uint8_t Can_Ch,uint8_t type) {
    CAN3__C620_and_C610_Type_select[Can_Ch-1] = type;
}

void Mo_Set_current(uint8_t type,uint8_t id,int16_t current,uint8_t* Tx_Buffer[]) {

};

void Mo_Callback(void) {

}