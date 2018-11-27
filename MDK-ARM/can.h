#include "stm32f4xx_hal.h"


//typedef struct 
//{
//	CAN_TxHeaderTypeDef*    TxMsg;
//	uint8_t Data[8];
//}CanTxMsgTypeDef ;

//typedef struct 
//{
//	CAN_RxHeaderTypeDef*    RxMsg;
//	uint8_t Data[8];
//}CanRxMsgTypeDef ;

//typedef struct 
//{
//	CAN_HandleTypeDef* hcan;
//	CanTxMsgTypeDef*   pTxMsg;
//	CanRxMsgTypeDef*  pRxMsg;
//}CANx_TypeDef;

uint8_t CAN_Send_Msg(uint8_t* msg,uint8_t len,uint32_t add);
uint8_t CAN_Receive_Msg(void);
