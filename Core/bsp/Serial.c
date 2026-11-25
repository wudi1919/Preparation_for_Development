//
// Created by Ken on 2026/10/19.
//

#include "Serial.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "PID_Calculator.h"


uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
volatile uint16_t rx_length = 0;
volatile uint8_t rx_flag = 0;

void Serial_Init(void) {
    // 调试阶段：先只发不收，避免 DMA 接收和阻塞发送冲突。
    // 以后需要接收时，再取消注释下面两行（并配合 UART_Data_Read 解析）。
    // HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART, rx_buffer, RX_BUFFER_SIZE);
    // __HAL_DMA_DISABLE_IT(SERIAL_UART.hdmarx, DMA_IT_HT);
}

void UART_Print_singal(uint8_t Byte) {
    HAL_UART_Transmit(&SERIAL_UART, (uint8_t *)&Byte, 1, HAL_MAX_DELAY);
}


int fputc(int ch, FILE *f)
{
    UART_Print_singal(ch);//将printf的底层重定向到自己的发送字节函数
    return ch;
}

void Print_Array(uint8_t* Array, uint8_t length) {
    for (uint8_t i = 0; i < length; i++) {
        UART_Print_singal(Array[i]);
    }
}//这里数组首地址也可以是字符串首地址

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart->Instance == USART4) {//写上串口判读好习惯
//         uint16_t pointer_Write_next =(pointer_Write + 1)%512;
//         if (pointer_Write_next == pointer_Read) {
//             HAL_UART_Receive_IT(&huart4, &RxData, 1);
//             return;
//         }//读取指针追上了写指针,放弃当前的数据
//         Data_buffer[pointer_Write] =   RxData;
//         pointer_Write = pointer_Write_next;
//         HAL_UART_Receive_IT(&huart4, &RxData, 1);
//     }
// }//这里是接收的函数还没有写好

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == SERIAL_UART_INST)
    {
        rx_length = Size;
        rx_flag = 1;
        HAL_UART_DMAStop(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);
        // 不在这里重开 DMA，等主循环 UART_Data_Read 解析完再重开，避免读的时候被新数据覆盖
    }
}

// void Serial_SendString(char *String)
void Serial_SendString(const char *String)
{
    int i;
    for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
    {
        UART_Print_singal(String[i]);		//依次调用Serial_SendByte发送每个字节数据
    }
}

// void Serial_Printf(char *format, ...)//这个是printf的重定义
void Serial_Printf(const char *format, ...)//这个是printf的重定义
{
    char String[256];				//定义字符数组
    va_list arg;					//定义可变参数列表数据类型的变量arg
    va_start(arg, format);			//从format开始，接收参数列表到arg变量
    // vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
    vsnprintf(String, sizeof(String), format, arg);	//使用vsnprintf限制长度，防止栈溢出
    va_end(arg);					//结束变量arg
    Serial_SendString(String);		//串口发送字符数组（字符串）
}

// uint8_t UART_Data_Read(uint8_t *data, uint8_t max_len)
// {
//     uint8_t cnt = 0;
//     while (cnt < max_len && pointer_Read != pointer_Write)
//     {
//         data[cnt++] = Data_buffer[pointer_Read];
//         pointer_Read = (pointer_Read + 1) % 512;
//     }
//     return cnt;
// }//这里是接收的函数还没有写好这个是版本1没有写好的




// void UART_Data_Read(void)//这里直接数据处理和帧头检测
// {
//
//
//     HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART, rx_buffer, RX_BUFFER_SIZE);
//     __HAL_DMA_DISABLE_IT(SERIAL_UART.hdmarx, DMA_IT_HT);//关闭半满中断
// }//目前串口接收暂时不需要

void PID_PFINT(PID_Data * p) {
    Serial_Printf("%f,%f,%f,%f\n", p->Target,p->Actual,p->Error0,p->Out);
}

/* 打印欧拉角（角度制），用指针传参避免拷贝 */
#define RAD_TO_DEG 57.2957795f   // 弧度转角度：180/π

void Serial_Print_Euler(const EulerAngle_t *angle) {
    if (angle == NULL) return;
    Serial_Printf("%f,%f,%f\n",
                  angle->yaw   * RAD_TO_DEG,
                  angle->pitch * RAD_TO_DEG,
                  angle->roll  * RAD_TO_DEG);
}

