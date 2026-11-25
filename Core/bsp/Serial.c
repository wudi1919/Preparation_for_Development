//
// Created by Ken on 2026/10/19.
//

#include "Serial.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "PID_Calculator.h"
#include"Motor_Contral.h"

/* rx_buffer 必须放在 DMA 可访问的 RAM_D2（见 .ld 的 .dma_buffer 段）：
 * 默认 .bss 在 DTCMRAM，而 DMA1/DMA2 物理上访问不到 DTCM，
 * 会导致 DMA 首字节传输即锁死、UART 接收永久死亡（实测抓到）。
 * aligned(32) 是为了 D-Cache 维护按缓存行对齐。 */
__attribute__((section(".dma_buffer"), aligned(32)))
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
volatile uint16_t rx_length = 0;
volatile uint8_t rx_flag = 0;
volatile uint32_t dbg_rx_evt = 0;   /* RX 空闲事件计数：调试串口接收是否工作 */

void Serial_Init(void) {
    /* PE7(UART7_RX) 在 CubeMX 里是 NOPULL：线不驱动时引脚浮空会随机进毛刺字节，
     * 触发 ORE → HAL 中止 DMA 并解除接收武装 → 串口接收永久死亡（实测抓到）。
     * 这里改为上拉，浮空时读回空闲高电平，从源头灭掉毛刺。 */
    GPIO_InitTypeDef rx_pull = {0};
    rx_pull.Pin       = GPIO_PIN_7;
    rx_pull.Mode      = GPIO_MODE_AF_PP;
    rx_pull.Alternate = GPIO_AF7_UART7;
    rx_pull.Pull      = GPIO_PULLUP;
    rx_pull.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &rx_pull);

    /* 清可能已锁存的错误/空闲标志，再武装接收 */
    __HAL_UART_CLEAR_OREFLAG(&SERIAL_UART);
    __HAL_UART_CLEAR_FEFLAG(&SERIAL_UART);
    __HAL_UART_CLEAR_NEFLAG(&SERIAL_UART);
    __HAL_UART_CLEAR_IDLEFLAG(&SERIAL_UART);
    /* 开启 DMA 串口接收，配合主循环 rx_flag + UART_Data_Read 解析 Kp/Ki/Kd */
    HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART, rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(SERIAL_UART.hdmarx, DMA_IT_HT);
}

/* 运行时 ORE/FE/NE 兜底：清错误标志并置 rx_flag，让主循环走 UART_Data_Read 重开接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == SERIAL_UART_INST) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        rx_length = 0;
        rx_flag = 1;
    }
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
        dbg_rx_evt++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        /* 不能在这里 HAL_UART_DMAStop：Stop 之后主循环再 ReceiveToIdle_DMA
         * 可能不再产生空闲事件，导致收完第一帧（比如开串口时的线路毛刺）后接收卡死。
         * 直接置标志位，等主循环 UART_Data_Read 解析完再重开 DMA 即可。 */
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




void UART_Data_Read(void)//这里直接数据处理和帧头检测
{
    /* 本工程没有开 D-Cache（无 SCB_EnableDCache 调用），CPU 直读 RAM_D2
     * 与 DMA 天然一致，不需要也不允许做 cache 维护：
     * 实测 D-Cache 关闭时写 DCIMVAC 会直接 BusFault 进 HardFault！ */

    /* 调参帧：0x55 0xAA + 3×int16（大端），值为 参数×100，分辨率 0.01
     * 格式：[0x55, 0xAA, Kp_hi, Kp_lo, Ki_hi, Ki_lo, Kd_hi, Kd_lo] 共 8 字节
     * 例：Kp=0.50, Ki=0.05, Kd=0.05 → 55 AA 00 32 00 05 00 05
     * 鲁棒解析：在缓冲里扫描帧头，容忍开串口毛刺等粘连在帧头前的噪声字节 */
    for (uint16_t i = 0; i + 8 <= rx_length && i < 32; i++) {
        if (rx_buffer[i] == 0x55 && rx_buffer[i + 1] == 0xAA) {
            int16_t kp100 = (int16_t)((rx_buffer[i + 2] << 8) | rx_buffer[i + 3]);
            int16_t ki100 = (int16_t)((rx_buffer[i + 4] << 8) | rx_buffer[i + 5]);
            int16_t kd100 = (int16_t)((rx_buffer[i + 6] << 8) | rx_buffer[i + 7]);
            float kp = kp100 / 100.0f;
            float ki = ki100 / 100.0f;
            float kd = kd100 / 100.0f;
            /* 增益合法性检查：防止噪声恰好拼出帧头后写入离谱参数 */
            if (kp >= 0.0f && kp <= 50.0f && ki >= 0.0f && ki <= 50.0f && kd >= 0.0f && kd <= 50.0f) {
                PID_config(&C610_stat[0][4].Speed, kp, ki, kd, 1000.0f, 3000.0f);
                Serial_Printf("OK Kp=%.2f Ki=%.2f Kd=%.2f\r\n", kp, ki, kd);
            }
            break;
        }
    }
    rx_length = 0;

    /* 重开前必须 AbortReceive 复位 RxState 并停掉旧 DMA：
     * ORE 等错误后 HAL 内部状态可能还停在 BUSY，直接 ReceiveToIdle_DMA 会
     * 返回 HAL_BUSY 静默失败（实测 CR1=0x0D，IDLEIE 丢失，接收永久死亡）。 */
    __HAL_UART_CLEAR_OREFLAG(&SERIAL_UART);
    HAL_UART_AbortReceive(&SERIAL_UART);
    HAL_UARTEx_ReceiveToIdle_DMA(&SERIAL_UART, rx_buffer, RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(SERIAL_UART.hdmarx, DMA_IT_HT);//关闭半满中断
}

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

