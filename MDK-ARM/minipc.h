#include "stm32f4xx_hal.h"
#include "BSP_Data.h"

#define TIME_MINIPCLOST 500000

typedef struct
{
	float Yaw_Now;	//未处理的值
	float Pitch_Now;
	uint32_t Distance_Now;	//单位：厘米
	float Yaw_Que[5];
	float Pitch_Que[5];
	uint32_t Distance_Que[10];
	float Yaw_Get;	
	float Pitch_Get;
	uint32_t Distance_Get;
	uint8_t Flag_Get;
	float Yaw_Real;
	float Pitch_Real;
	float Yaw_Zero;
	float Pitch_Zero;
	uint8_t Enemy_Flag;				//是否识别到敌人 0：无  1：有
	uint8_t Enemy_Distance;			//敌人距离	除以10后单位m
	uint8_t Used_Flag;		//妙算信息被使用过的标志 1：使用过   0：未使用
	uint32_t TimeStamp;		//识别成功的时间戳
}MiniPC_Res;

extern void Mainifold_Res_Task(void);
extern CIRCLE_BUFF_t Que_Mainifold ;
extern void Send_MiniPC(void);
extern MiniPC_Res MiniPC_Data;