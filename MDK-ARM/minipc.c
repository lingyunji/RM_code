#include "minipc.h"
#include "BSP_Data.h"
#include "math.h"
#include "imu.h"
#include "angle.h"
#include "q6623.h"

MiniPC_Res MiniPC_Data={0,0,{0},{0},0,0,0,0,0,0,0,0,0,0,0};
CIRCLE_BUFF_t Que_Mainifold = {0, 0, {0}};
extern UART_HandleTypeDef huart6;
static uint8_t Res_OK = 0;//接收成功标志
/**
  * @brief  miniPC数据滤波
  * @param  uint8_t data
  * @retval None
  */
static void MiniPC_Smooth(void)
{
	static uint8_t i = 0;
	static float sumY = 0, sumP = 0;
	static uint32_t sumD = 0;

	for (i = 1; i < 3; i++)
	{
		MiniPC_Data.Yaw_Que[i - 1] = MiniPC_Data.Yaw_Que[i];
		MiniPC_Data.Pitch_Que[i - 1] = MiniPC_Data.Pitch_Que[i];
	}

	for (i = 1; i < 9; i++)
	{
		MiniPC_Data.Distance_Que[i - 1] = MiniPC_Data.Distance_Que[i];
	}

	MiniPC_Data.Yaw_Que[2] = MiniPC_Data.Yaw_Now;
	MiniPC_Data.Pitch_Que[2] = MiniPC_Data.Pitch_Now;

	MiniPC_Data.Distance_Que[8] = MiniPC_Data.Distance_Now;

	for (i = 0, sumY = 0, sumP = 0; i < 3; i++)
	{
		sumY += MiniPC_Data.Yaw_Que[i];
		sumP += MiniPC_Data.Pitch_Que[i];
	}

	for (i = 0, sumD = 0; i < 9; i++)
	{
		sumD += MiniPC_Data.Distance_Que[i];
	}

	MiniPC_Data.Yaw_Get = sumY / 3.0f;
	MiniPC_Data.Pitch_Get = sumP / 3.0f;
	MiniPC_Data.Distance_Get = sumD / 8.0f;
}
static uint8_t frame_temp_mini[30];
/**
  * @brief  串口接受数据处理
  * @param  uint8_t data
  * @retval None
  */
void Mainifold_Res_Task(void)
{
	while(bufferlen(&Que_Mainifold) != 0)
	{
		//寻找帧头AD
		if(bufferPop(&Que_Mainifold,frame_temp_mini) == 0xFF)  
		{
			buffer_multiPop(&Que_Mainifold, &frame_temp_mini[1],6);
			if(bufferPop(&Que_Mainifold,&frame_temp_mini[7]) == 0xFE)	
			{
				Res_OK = 1;
			}
		}
		if (Res_OK)
		{
			MiniPC_Data.Yaw_Now = ((int16_t)(frame_temp_mini[3] << 8 | frame_temp_mini[2])) / 100.0f;
			MiniPC_Data.Pitch_Now = ((int16_t)(frame_temp_mini[5] << 8 | frame_temp_mini[4])) / 100.0f;
			MiniPC_Data.Distance_Now =  frame_temp_mini[6];
//			if(MiniPC_Data.Yaw_Now
			MiniPC_Data.Flag_Get = frame_temp_mini[1];
			if (MiniPC_Data.Flag_Get != 0 && fabs(MiniPC_Data.Yaw_Real-MiniPC_Data.Yaw_Now) < 40)
			{
				MiniPC_Data.Enemy_Flag = MiniPC_Data.Flag_Get;
				if(MiniPC_Data.Flag_Get == 1)
				{
						if(
						(fabs(MiniPC_Data.Yaw_Real-MiniPC_Data.Yaw_Get) < 8.0 && fabs(MiniPC_Data.Pitch_Real - MiniPC_Data.Pitch_Get) < 8.0)
						|| Get_Time_Micros() - MiniPC_Data.TimeStamp > 100000)
						{
							MiniPC_Smooth();
							MiniPC_Data.Yaw_Real   = MiniPC_Data.Yaw_Get;
							MiniPC_Data.Pitch_Real = MiniPC_Data.Pitch_Get;
							MiniPC_Data.Enemy_Distance = frame_temp_mini[6] / 100.0f;
							MiniPC_Data.TimeStamp = Get_Time_Micros();//记录成功识别的时间
							MiniPC_Data.Used_Flag = 0;
						}
				}
				else
				{
					MiniPC_Smooth();
					MiniPC_Data.Yaw_Real   = MiniPC_Data.Yaw_Get;
					MiniPC_Data.Pitch_Real = MiniPC_Data.Pitch_Get;
//						MiniPC_Data.Yaw_Real   = MiniPC_Data.Yaw_Now;//删除过滤
//						MiniPC_Data.Pitch_Real = MiniPC_Data.Pitch_Now;
					MiniPC_Data.Enemy_Distance = MiniPC_Data.Distance_Get / 100.0f;
					MiniPC_Data.TimeStamp = Get_Time_Micros();//记录成功识别的时间
					MiniPC_Data.Used_Flag = 0;
				}
			}
			else if (Get_Time_Micros() - MiniPC_Data.TimeStamp > TIME_MINIPCLOST)
			{
				MiniPC_Data.Enemy_Flag = MiniPC_Data.Flag_Get;
			}
//			if ((frame_temp_mini[1] != 0 && (abs(MiniPC_Data.Yaw_Real-(((int16_t)(frame_temp_mini[3]<<8 | frame_temp_mini[2]))/100.0f)) < 1.0) && (abs(MiniPC_Data.Pitch_Real-(((int16_t)(frame_temp_mini[5]<<8 | frame_temp_mini[4]))/100.0f)) < 1.0)) || Get_Time_Micros() - MiniPC_Data.TimeStamp > 200000)
//			{
//				MiniPC_Data.Yaw_Real   = ((int16_t)(frame_temp_mini[3]<<8 | frame_temp_mini[2]))/100.0f;
//				MiniPC_Data.Pitch_Real = ((int16_t)(frame_temp_mini[5]<<8 | frame_temp_mini[4]))/100.0f;
//				MiniPC_Data.Enemy_Distance = frame_temp_mini[6] / 10.0f;
//				MiniPC_Data.TimeStamp = Get_Time_Micros();//记录成功识别的时间
//				MiniPC_Data.Used_Flag = 0;
//			}
			//MiniPCFrameCounter++;
		}
	}
}
CIRCLE_BUFF_t Que_MiniPC_Tx  = {0, 0, {0}};
int16_t Yaw_Send,Pitch_Send;
//串口DMA配置完之后,直接调用函数 每调用一次即发送一帧
void Send_MiniPC(void)
{
	uint8_t Send_MiniPC[50] = {0};			//准备即将要发送的数据
	uint8_t* pdata = Send_MiniPC;
	static uint8_t MiniPC_DMA_BUFF[200] = {0}; //DMA取值缓冲区
	uint8_t cnt = 0;

//	static int16_t Yaw_Send,Pitch_Send;
	Yaw_Send=Angle_Process(yaw_angle)*100;
//	Pitch_Send=Angle_Process(GMPitchEncoder.ecd_angle-MiniPC_Data.Pitch_Zero)*100;
	Pitch_Send=Angle_Process(GMPitchEncoder.ecd_angle)*100;
	//商量好数据帧后直接填进去
	Send_MiniPC[cnt++] = 0xDA;
	Send_MiniPC[cnt++] = (uint8_t)(Yaw_Send>>8);
	Send_MiniPC[cnt++] = (uint8_t)Yaw_Send;
	Send_MiniPC[cnt++] = (uint8_t)(Pitch_Send>>8);
	Send_MiniPC[cnt++] = (uint8_t)Pitch_Send;
		Send_MiniPC[cnt++] = 0x00;
	Send_MiniPC[cnt++] = 0xDB;
	for(int i = 0;i<cnt;i++)
	{
		bufferPush(&Que_MiniPC_Tx, pdata[i]);//数据写入队列
	}
	
	//


	uint8_t Protocol_len = bufferlen(&Que_MiniPC_Tx);//队列数据长度
	buffer_multiPop(&Que_MiniPC_Tx, MiniPC_DMA_BUFF, Protocol_len);//从队列中取出所有数据
	HAL_UART_Transmit_DMA(&huart6, MiniPC_DMA_BUFF, Protocol_len);//启用DMA发送  串口待定huartx
}
