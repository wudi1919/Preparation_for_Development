#include "bsp_fdcan.h"
#include "bsp_Can_Callback.h"
/**
************************************************************************
* @brief:      	bsp_can_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN 使能
************************************************************************
**/
void bsp_can_init(FDCAN_HandleTypeDef *hfdcan)
{
	can_filter_init(hfdcan);
	HAL_FDCAN_Start(hfdcan);                               //开启FDCAN
	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_WATERMARK, 0);
}
/**
************************************************************************
* @brief:      	bsp_fdcan_set_baud
* @param:       hfdcan：FDCAN 句柄
* @param:       mode：CAN_CLASS（经典 CAN）或 CAN_FD_BRS（FD，数据域 BRS 使能）
* @param:       baud：波特率枚举 CAN_BR_xxx
* @retval:      void
* @details:    	运行时切换帧格式与波特率。
*               内核时钟 120MHz（PLL1Q = 480/4），实际速率 = 120M / (brp*(1+seg1+seg2))。
*               FDOE/BRSE 只能在 INIT+CCE 下修改，所以先 DeInit 再 Init。
*               Init 之后必须再调 bsp_can_init() 重新配置滤波器并启动。
************************************************************************
**/
void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
    uint32_t nom_brp = 0, nom_seg1 = 0, nom_seg2 = 0, nom_sjw = 0;  /* 仲裁域参数 */
    uint32_t dat_brp = 0, dat_seg1 = 0, dat_seg2 = 0, dat_sjw = 0;  /* 数据域参数 */

    /* 经典 CAN：仲裁域用指定波特率，数据域只为占位不会用到 */
    if (mode == CAN_CLASS) {
        switch (baud) {
            case CAN_BR_125K: nom_brp = 6; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break; /* 125K，采样点 87.5% */
            case CAN_BR_200K: nom_brp = 3; nom_seg1 = 174; nom_seg2 = 25; nom_sjw = 25; break; /* 200K，采样点 87.5% */
            case CAN_BR_250K: nom_brp = 3; nom_seg1 = 139; nom_seg2 = 20; nom_sjw = 20; break; /* 250K，采样点 87.5% */
            case CAN_BR_500K: nom_brp = 1; nom_seg1 = 209; nom_seg2 = 30; nom_sjw = 30; break; /* 500K，采样点 87.5% */
            case CAN_BR_1M:   nom_brp = 1; nom_seg1 = 89;  nom_seg2 = 30; nom_sjw = 30; break; /* 1M，采样点 75% */
            default:          nom_brp = 3; nom_seg1 = 29;  nom_seg2 = 10; nom_sjw = 10; break; /* 默认 1M */
        }
        dat_brp = 1; dat_seg1 = 29; dat_seg2 = 10; dat_sjw = 10;
        hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    }
    /* CAN FD（带 BRS）：仲裁域固定 1M，数据域用指定高速波特率 */
    else if (mode == CAN_FD_BRS) {
        switch (baud) {
            case CAN_BR_2M:  dat_brp = 1; dat_seg1 = 44; dat_seg2 = 15; dat_sjw = 15; break; /* 2.0M，采样点 75% */
            case CAN_BR_2M5: dat_brp = 1; dat_seg1 = 47; dat_seg2 = 12; dat_sjw = 12; break; /* 2.5M，采样点 80% */
            case CAN_BR_3M2: dat_brp = 1; dat_seg1 = 28; dat_seg2 = 9;  dat_sjw = 9;  break; /* 3.16M，采样点 76.3% */
            case CAN_BR_4M:  dat_brp = 1; dat_seg1 = 23; dat_seg2 = 6;  dat_sjw = 6;  break; /* 4.0M，采样点 80% */
            case CAN_BR_5M:  dat_brp = 1; dat_seg1 = 20; dat_seg2 = 3;  dat_sjw = 3;  break; /* 5.0M，采样点 87.5% */
            default:         dat_brp = 1; dat_seg1 = 44; dat_seg2 = 15; dat_sjw = 15; break; /* 默认 2M */
        }
        nom_brp = 1; nom_seg1 = 89; nom_seg2 = 30; nom_sjw = 30;
        hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    }
    else {
        return;
    }

    /* 先反初始化，再写新参数，最后初始化 */
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
************************************************************************
* @brief:      	bsp_can_switch
* @param:       hfdcan：FDCAN 句柄
* @param:       mode：CAN_CLASS 或 CAN_FD_BRS
* @param:       baud：波特率枚举 CAN_BR_xxx
* @retval:      void
* @details:    	切换工作模式并一步完成启动，等价于 set_baud + bsp_can_init。
*               两段必须分开：滤波器与 Start 只能在 Init 之后做。
************************************************************************
**/
void bsp_can_switch(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
    bsp_fdcan_set_baud(hfdcan, mode, baud);   /* DeInit + 配时序 + Init */
    bsp_can_init(hfdcan);                     /* 滤波器 + Start + 使能中断 */
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void can_filter_init(FDCAN_HandleTypeDef *hfdcan)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //标准ID
	fdcan_filter.FilterIndex = 0;                                  //滤波器索引                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	HAL_FDCAN_ConfigFilter(hfdcan,&fdcan_filter); 		 				  //接收ID2
	//拒绝接收匹配不成功的标准ID和扩展ID,不接受远程帧
	HAL_FDCAN_ConfigGlobalFilter(hfdcan,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO0, 1);
//	HAL_FDCAN_ConfigFifoWatermark(hfdcan, FDCAN_CFG_RX_FIFO1, 1);
//	HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0);
}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan：FDCAN句柄
* @param:       id：CAN设备ID
* @param:       data：发送的数据
* @param:       len：发送的数据长度
* @retval:     	void
* @details:    	发送数据
************************************************************************
**/
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8)
		pTxHeader.DataLength = len;
	if(len==12)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	if(len==16)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	if(len==20)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	if(len==24)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	if(len==32)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	if(len==48)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	if(len==64)
		pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    /* 按当前总线模式选帧格式：经典 CAN 关 BRS/FD，CAN FD 开 BRS/FD */
    if (hfdcan->Init.FrameFormat == FDCAN_FRAME_CLASSIC) {
        pTxHeader.BitRateSwitch = FDCAN_BRS_OFF;
        pTxHeader.FDFormat      = FDCAN_CLASSIC_CAN;
    } else {
        pTxHeader.BitRateSwitch = FDCAN_BRS_ON;
        pTxHeader.FDFormat      = FDCAN_FD_CAN;
    }
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) 
		return 1;//发送
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan：FDCAN句柄
* @param:       buf：接收数据缓存
* @retval:     	接收的数据长度
* @details:    	接收数据
************************************************************************
**/
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8)
			len = pRxHeader.DataLength;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_12)
			len = 12;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_16)
			len = 16;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_20)
			len = 20;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_24)
			len = 24;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_32)
			len = 32;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_48)
			len = 48;
		else if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_64)
			len = 64;
		
		return len;//接收数据
	}
	return 0;	
}



uint8_t rx_data1[64] = {0};
uint16_t rec_id1;
void fdcan1_rx_callback(void)
{
	
	if (fdcanx_receive(&hfdcan1, &rec_id1, rx_data1)) Can_Data_Processing_DJI(1);
}
uint8_t rx_data2[64] = {0};
uint16_t rec_id2;
void fdcan2_rx_callback(void)
{

	if (fdcanx_receive(&hfdcan2, &rec_id2, rx_data2)) Can_Data_Processing_DJI(2);
}
uint8_t rx_data3[64] = {0};
uint16_t rec_id3;
void fdcan3_rx_callback(void)
{

	if (fdcanx_receive(&hfdcan3, &rec_id3, rx_data3)) Can_Data_Processing_DJI(3);
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
	{
		fdcan1_rx_callback();
	}
	if(hfdcan == &hfdcan2)
	{
		fdcan2_rx_callback();
	}
	if(hfdcan == &hfdcan3)
	{
		fdcan3_rx_callback();
	}
}

