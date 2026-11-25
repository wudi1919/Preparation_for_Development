//
// Created by Ken on 2026/10/19.
//

#ifndef PWM_CHASSIS_CONTROL_SERIAL_PRINT_H
#define PWM_CHASSIS_CONTROL_SERIAL_PRINT_H

#include <stdint.h>
#include"PID_Calculator.h"
#include "imu_quaternion.h"

#define RX_BUFFER_SIZE 256

/* 串口选择：改成当前 CubeMX 里配置的那个串口（句柄 + 外设实例）。
 * 现在配置的是 UART7（PE7=RX, PE8=TX），所以填 huart7 / UART7。
 * 以后换串口只要改这两行，不用再去改 .c 里的每一处。 */
#define SERIAL_UART      huart7
#define SERIAL_UART_INST UART7

extern uint8_t  rx_buffer[RX_BUFFER_SIZE];
extern volatile uint16_t rx_length;
extern volatile uint8_t  rx_flag;
extern volatile uint32_t dbg_rx_evt;

void Serial_Init(void);
void UART_Print_singal(uint8_t Byte);
void Print_Array(uint8_t *Array, uint8_t length);
void Serial_SendString(const char *String);
void Serial_Printf(const char *format, ...);
void UART_Data_Read(void);
void PID_PFINT(PID_Data * p);
void PID_PFINT_ALL(void);
void Serial_Print_Euler(const EulerAngle_t *angle);
#endif //PWM_CHASSIS_CONTROL_SERIAL_PRINT_H
