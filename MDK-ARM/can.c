#include "stm32f4xx_hal.h"
#include "can.h"
#include "stdio.h"

//CANx_TypeDef hcan1;
//CanTxMsgTypeDef     TxMessage;      //发送消息
//CanRxMsgTypeDef     RxMessage;      //接收消息
extern CAN_HandleTypeDef hcan1;

CAN_RxHeaderTypeDef   RxMsg;
CAN_TxHeaderTypeDef   TxMsg;
uint8_t buf[8];

   

uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len,uint32_t add)
{
	uint16_t i=0;
	TxMsg.StdId=0x1FF;
	TxMsg.IDE=CAN_ID_STD;
	TxMsg.RTR=CAN_RTR_DATA;
	TxMsg.DLC=len;
//	for(i=0;i<len;i++)
//	hcan1.pTxMsg->Data[i]=msg[i];
	HAL_CAN_AddTxMessage(&hcan1,&TxMsg,msg,(uint32_t*)add);
	return 0;
}
uint8_t CAN_Receive_Msg(void)
{
	uint32_t i;
	
	if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMsg,buf)!=HAL_OK ) return 1;
	return 0;
}


