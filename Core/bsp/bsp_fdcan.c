#include "bsp_fdcan.h"

/**
 * @brief   CAN 外设初始化（配置滤波器、启动并激活中断）
 * @param   hfdcan：FDCAN 句柄
 * @note    必须在 bsp_fdcan_set_baud() 之后调用，用于重新配置滤波器并启动外设
 */
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan)
{
    can_filter_init(hfdcan);                       // 配置接收滤波器
    HAL_FDCAN_Start(hfdcan);                       // 启动 FDCAN（进入正常模式）
    HAL_FDCAN_ActivateNotification(hfdcan,
        FDCAN_IT_RX_FIFO0_WATERMARK                // RX FIFO0 水位线中断
        | FDCAN_IT_TX_COMPLETE                     // 发送完成中断
        | FDCAN_IT_TX_FIFO_EMPTY                   // TX FIFO 空中断
        | FDCAN_IT_BUS_OFF                         // 总线关闭中断
        | FDCAN_IT_ARB_PROTOCOL_ERROR              // 仲裁段协议错误
        | FDCAN_IT_DATA_PROTOCOL_ERROR             // 数据段协议错误
        | FDCAN_IT_ERROR_PASSIVE                   // 错误被动
        | FDCAN_IT_ERROR_WARNING,                  // 错误警告
        0);                                        // 使用 TX FIFO 队列，无需 buffer 索引
}

/**
 * @brief   CAN 接收滤波器初始化
 * @param   hfdcan：FDCAN 句柄
 * @note    掩码模式下 FilterID1=ID、FilterID2=掩码；此处均置 0，表示接收所有标准 ID
 */
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
    FDCAN_FilterTypeDef fdcan_filter;

    fdcan_filter.IdType = FDCAN_STANDARD_ID;          // 只过滤标准 ID（11 位）
    fdcan_filter.FilterIndex = 0;                     // 使用第 0 个过滤器单元
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;      // 掩码模式
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 匹配帧放入 RX FIFO0
    fdcan_filter.FilterID1 = 0x00;                    // 期望 ID（掩码为 0 时无效）
    fdcan_filter.FilterID2 = 0x00;                    // 掩码：0 表示不关心任何位

    HAL_FDCAN_ConfigFilter(hfdcan, &fdcan_filter);

    // 拒绝所有未匹配的标准帧、扩展帧以及远程帧
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // RX FIFO0 水位线设为 1：收到 1 条消息即触发水位线中断
    HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
}

/**
 * @brief   设置 FDCAN 的波特率与工作模式（运行时切换标准 CAN / CAN FD）
 * @param   hfdcan：FDCAN 句柄
 * @param   mode：CAN_CLASS（经典 CAN）或 CAN_FD_BRS（FD 模式，数据域 BRS 使能）
 * @param   baud：波特率枚举值（CAN_BR_xxx）
 * @note    FDCAN 内核时钟为 120MHz（PLL1Q = 480/4）。
 *          切换会先 DeInit 再 Init（因为 CCCR 的 FDOE/BRSE 位只能在 INIT+CCE 下修改）。
 *          切换后必须再次调用 bsp_can_init() 重新配置滤波器并启动。
 *          注意：本函数不改 element size，FD 模式 payload 需 ≤8 字节（大疆/达妙协议均为 8 字节）。
 */
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
    uint32_t nom_brp = 0, nom_seg1 = 0, nom_seg2 = 0, nom_sjw = 0; // 仲裁域参数
    uint32_t dat_brp = 0, dat_seg1 = 0, dat_seg2 = 0, dat_sjw = 0; // 数据域参数

    /* 经典 CAN 模式：仲裁域使用指定波特率，数据域占位（不会用到） */
    if (mode == CAN_CLASS)
    {
        switch (baud)
        {
            // 实际波特率 = 120MHz / (brp*(1+seg1+seg2))
            case CAN_BR_125K: nom_brp = 6; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break; // 125K，采样点 87.5%
            case CAN_BR_200K: nom_brp = 3; nom_seg1 = 174; nom_seg2 = 25; nom_sjw = 25; break; // 200K，采样点 87.5%
            case CAN_BR_250K: nom_brp = 3; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break; // 250K，采样点 87.5%
            case CAN_BR_500K: nom_brp = 1; nom_seg1 = 209; nom_seg2 = 30; nom_sjw = 30; break; // 500K，采样点 87.5%
            case CAN_BR_1M:   nom_brp = 1; nom_seg1 = 89;  nom_seg2 = 30; nom_sjw = 30; break; // 1M，采样点 75%
            default:          nom_brp = 3; nom_seg1 = 29;  nom_seg2 = 10; nom_sjw = 10; break; // 默认 1M
        }
        dat_brp = 1; dat_seg1 = 29; dat_seg2 = 10; dat_sjw = 10;
        hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    }

    /* CAN FD（带 BRS）：仲裁域固定 1M，数据域使用指定高速波特率 */
    if (mode == CAN_FD_BRS)
    {
        switch (baud)
        {
            case CAN_BR_2M:  dat_brp = 1; dat_seg1 = 44; dat_seg2 = 15; dat_sjw = 15; break; // 2.0M，采样点 75%
            case CAN_BR_2M5: dat_brp = 1; dat_seg1 = 47; dat_seg2 = 12; dat_sjw = 12; break; // 2.5M，采样点 80%
            case CAN_BR_3M2: dat_brp = 1; dat_seg1 = 28; dat_seg2 = 9;  dat_sjw = 9;  break; // 3.158M，采样点 76.3%
            case CAN_BR_4M:  dat_brp = 1; dat_seg1 = 23; dat_seg2 = 6;  dat_sjw = 6;  break; // 4.0M，采样点 80%
            case CAN_BR_5M:  dat_brp = 1; dat_seg1 = 20; dat_seg2 = 3;  dat_sjw = 3;  break; // 5.0M，采样点 87.5%
            default:         dat_brp = 1; dat_seg1 = 44; dat_seg2 = 15; dat_sjw = 15; break; // 默认 2M
        }
        nom_brp = 1; nom_seg1 = 89; nom_seg2 = 30; nom_sjw = 30;       // 仲裁域固定 1M
        hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    }

    // 先反初始化，再写新参数，最后初始化
    HAL_FDCAN_DeInit(hfdcan);

    hfdcan->Init.NominalPrescaler     = nom_brp;
    hfdcan->Init.NominalTimeSeg1      = nom_seg1;
    hfdcan->Init.NominalTimeSeg2      = nom_seg2;
    hfdcan->Init.NominalSyncJumpWidth = nom_sjw;

    hfdcan->Init.DataPrescaler        = dat_brp;
    hfdcan->Init.DataTimeSeg1         = dat_seg1;
    hfdcan->Init.DataTimeSeg2         = dat_seg2;
    hfdcan->Init.DataSyncJumpWidth    = dat_sjw;

    HAL_FDCAN_Init(hfdcan);
}

/**
 * @brief   FDCAN 发送数据（根据当前模式自动适配经典 CAN / CAN FD）
 * @param   hfdcan：FDCAN 句柄
 * @param   id：CAN 标识符（标准 11 位）
 * @param   data：发送数据缓冲区
 * @param   len：数据长度（经典 CAN 最大 8，FD 支持 8/12/16/20/24/32/48/64）
 * @retval  0 成功，1 失败
 */
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader;

    // 公共帧头
    pTxHeader.Identifier = id;
    pTxHeader.IdType = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker = 0;

    // 根据当前工作模式配置帧格式
    if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC)
    {
        // 经典 CAN：BRS 关闭，FD 格式关闭，最多 8 字节
        pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        pTxHeader.FDFormat = FDCAN_CLASSIC_CAN;
        if (len > 8) return 1;
        pTxHeader.DataLength = len;
    }
    else
    {
        // CAN FD（带 BRS）：BRS 开启，FD 格式开启
        pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
        pTxHeader.FDFormat = FDCAN_FD_CAN;

        if (len <= 8)
            pTxHeader.DataLength = len;
        else if (len == 12)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
        else if (len == 16)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
        else if (len == 20)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
        else if (len == 24)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
        else if (len == 32)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
        else if (len == 48)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
        else if (len == 64)
            pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
        else
            return 1;   // 不支持的数据长度
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
        return 1;   // 队列满或参数错误
    return 0;
}

/**
 * @brief   FDCAN 接收数据（从 RX FIFO0 读取）
 * @param   hfdcan：FDCAN 句柄
 * @param   rec_id：返回接收到的 CAN ID
 * @param   buf：接收数据缓冲区
 * @retval  接收数据长度（字节），0 表示无新消息
 */
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader;
    uint8_t len;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) == HAL_OK)
    {
        *rec_id = pRxHeader.Identifier;

        // 根据 DLC 换算实际字节数
        if (pRxHeader.DataLength <= FDCAN_DLC_BYTES_8)
            len = pRxHeader.DataLength;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_12)
            len = 12;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_16)
            len = 16;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_20)
            len = 20;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_24)
            len = 24;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_32)
            len = 32;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_48)
            len = 48;
        else if (pRxHeader.DataLength == FDCAN_DLC_BYTES_64)
            len = 64;

        return len;
    }
    return 0;
}

/* 各 FDCAN 口的接收缓冲与 ID */
uint8_t rx_data1[64] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
    fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
}

uint8_t rx_data2[64] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{
    fdcanx_receive(&hfdcan2, &rec_id2, rx_data2);
}

uint8_t rx_data3[64] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{
    fdcanx_receive(&hfdcan3, &rec_id3, rx_data3);
}

/* RX FIFO0 水位线中断回调：按句柄分发到对应口的回调 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == &hfdcan1)
        fdcan1_rx_callback();
    if (hfdcan == &hfdcan2)
        fdcan2_rx_callback();
    if (hfdcan == &hfdcan3)
        fdcan3_rx_callback();
}

/**
 * @brief   FDCAN 错误状态回调（总线关闭、错误被动、错误警告等）
 * @note    总线关闭时清除 INIT 位，让控制器自动恢复通信
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    // 总线关闭错误：清除 INIT 位，让 CAN 控制器自动恢复
    if (ErrorStatusITs & FDCAN_IR_BO)
    {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }

    // 错误被动中断（预留，可按需重新初始化）
    if (ErrorStatusITs & FDCAN_IR_EP)
    {
        // 可在此处做重初始化等处理
    }
}
