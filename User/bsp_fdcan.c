#include "bsp_fdcan.h"

/* --- 新增：为 CAN2 定义一个弱回调函数 --- */
__weak void fdcan1_rx_callback(void) {}
__weak void fdcan2_rx_callback(void) {} // 新增 CAN2 专用

/**
* @brief:      	bsp_can_init(void)
* @details:    	CAN 使能与中断开启
**/
void bsp_can_init(void)
{
	can_filter_init();
	
	HAL_FDCAN_Start(&hfdcan1);
	HAL_FDCAN_Start(&hfdcan2); // 开启 CAN2

	/* --- 关键点 1：必须为 CAN2 也开启中断通知 --- */
	uint32_t active_it = FDCAN_IT_RX_FIFO0_WATERMARK | FDCAN_IT_TX_COMPLETE | 
						 FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF | 
						 FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR | 
						 FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING;

	HAL_FDCAN_ActivateNotification(&hfdcan1, active_it, 0x00000F00);
	HAL_FDCAN_ActivateNotification(&hfdcan2, active_it, 0x00000F00); // 新增：为 CAN2 开启中断
}

/**
* @brief:      	can_filter_init(void)
* @details:    	CAN 滤波器初始化
**/
void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	
	fdcan_filter.IdType = FDCAN_STANDARD_ID;
	fdcan_filter.FilterIndex = 0;                   
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;

	/* --- 关键点 2：必须分别为 CAN1 和 CAN2 配置滤波器 --- */
	// 配置 CAN1
	HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter); 
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);

	// 配置 CAN2 (如果不配置，CAN2 会默认丢弃所有收到的包)
	HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter); 
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);
}

/**
* @brief:      	HAL_FDCAN_RxFifo0Callback
* @details:    	接收中断回调
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* --- 关键点 3：识别中断来源 --- */
    if (hfdcan->Instance == FDCAN1)
    {
		fdcan1_rx_callback();
    }
	else if (hfdcan->Instance == FDCAN2)
	{
		fdcan2_rx_callback(); // 如果是 CAN2 触发，执行 CAN2 的回调
	}
}

/* ------------------------------------------------------------------
   以下函数 (set_baud, send_data, receive) 保持不变，
   因为它们通过传入的句柄指针(hfdcan)来工作，是通用的。
------------------------------------------------------------------ */

void bsp_fdcan_set_baud(hcan_t *hfdcan, uint8_t mode, uint8_t baud)
{
	uint32_t nom_brp=0, nom_seg1=0, nom_seg2=0, nom_sjw=0;
	uint32_t dat_brp=0, dat_seg1=0, dat_seg2=0, dat_sjw=0;
	
	if(mode == CAN_CLASS)
	{
		switch (baud)
		{
			case CAN_BR_125K: 	nom_brp=4 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; 
			case CAN_BR_200K: 	nom_brp=2 ; nom_seg1=174; nom_seg2=25; nom_sjw=25; break; 
			case CAN_BR_250K: 	nom_brp=2 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; 
			case CAN_BR_500K: 	nom_brp=1 ; nom_seg1=139; nom_seg2=20; nom_sjw=20; break; 
			case CAN_BR_1M:		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; break; 
		}
		dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10;
		hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	}
	if(mode == CAN_FD_BRS)
	{
		switch (baud)
		{
			case CAN_BR_2M: 	dat_brp=1 ; dat_seg1=29; dat_seg2=10; dat_sjw=10; break;
			case CAN_BR_2M5: 	dat_brp=1 ; dat_seg1=25; dat_seg2=6 ; dat_sjw=6 ; break;
			case CAN_BR_3M2: 	dat_brp=1 ; dat_seg1=19; dat_seg2=5 ; dat_sjw=5 ; break;
			case CAN_BR_4M: 	dat_brp=1 ; dat_seg1=14; dat_seg2=5 ; dat_sjw=5 ; break;
			case CAN_BR_5M:		dat_brp=1 ; dat_seg1=13; dat_seg2=2 ; dat_sjw=2 ; break;
		}
		nom_brp=1 ; nom_seg1=59 ; nom_seg2=20; nom_sjw=20; 
		hfdcan->Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	}
	
	HAL_FDCAN_DeInit(hfdcan);
	hfdcan->Init.NominalPrescaler = nom_brp;
	hfdcan->Init.NominalTimeSeg1  = nom_seg1;
	hfdcan->Init.NominalTimeSeg2  = nom_seg2;
	hfdcan->Init.NominalSyncJumpWidth = nom_sjw;
	hfdcan->Init.DataPrescaler = dat_brp;
	hfdcan->Init.DataTimeSeg1  = dat_seg1;
	hfdcan->Init.DataTimeSeg2  = dat_seg2;
	hfdcan->Init.DataSyncJumpWidth = dat_sjw;
	HAL_FDCAN_Init(hfdcan);
}

uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{	
    FDCAN_TxHeaderTypeDef pTxHeader;
    pTxHeader.Identifier=id;
    pTxHeader.IdType=FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType=FDCAN_DATA_FRAME;
	
	if(len<=8) pTxHeader.DataLength = len;
	else if(len==12) pTxHeader.DataLength = FDCAN_DLC_BYTES_12;
	else if(len==16) pTxHeader.DataLength = FDCAN_DLC_BYTES_16;
	else if(len==20) pTxHeader.DataLength = FDCAN_DLC_BYTES_20;
	else if(len==24) pTxHeader.DataLength = FDCAN_DLC_BYTES_24;
	else if(len==32) pTxHeader.DataLength = FDCAN_DLC_BYTES_32;
	else if(len==48) pTxHeader.DataLength = FDCAN_DLC_BYTES_48;
	else if(len==64) pTxHeader.DataLength = FDCAN_DLC_BYTES_64;
	
    pTxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch=FDCAN_BRS_ON;
    pTxHeader.FDFormat=FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker=0;
 
	if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data)!=HAL_OK) return 1;
	return 0;	
}

uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf)
{	
	FDCAN_RxHeaderTypeDef pRxHeader;
	uint8_t len;
	
	if(HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &pRxHeader, buf)==HAL_OK)
	{
		*rec_id = pRxHeader.Identifier;
		if(pRxHeader.DataLength<=FDCAN_DLC_BYTES_8) len = pRxHeader.DataLength;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_12) len = 12;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_16) len = 16;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_20) len = 20;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_24) len = 24;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_32) len = 32;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_48) len = 48;
		else if(pRxHeader.DataLength==FDCAN_DLC_BYTES_64) len = 64;
		return len;
	}
	return 0;	
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	if(ErrorStatusITs & FDCAN_IR_BO)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}
