#include "stm32f4xx_hal.h"
#include "can.h"
#include "bsp_uart.h"
#include "q6623.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_RxHeaderTypeDef   RxMsg;
extern CAN_TxHeaderTypeDef   TxMsg;
extern uint8_t buf[8];

int v=0;
uint8_t  pitch[8]={0};
uint8_t  yaw[8]={0};
int32_t yaw_counter[3]={0};
int32_t pitch_counter[3]={0};
int32_t pitch_last[3]={0};
int32_t yaw_last[3]={0};
int32_t pitch_diff;
int32_t yaw_diff;
int32_t roll=0;
int32_t pitch_relative;
int32_t yaw_relative;
float  pitch_abs;
float yaw_abs;
int32_t pitch_start,yaw_start;
int32_t pitch_speed=1000;
int32_t yaw_speed=1000;
int32_t pitch_speedlast=0;
int32_t yaw_speedlast=0;
int32_t pitch_value,yaw_value=0;

volatile Encoder GMYawEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile Encoder GMPitchEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};

void Motor_CAN_Send()
{
	pitch_speed=rc.ch1*3;
	yaw_speed=rc.ch2*3;
	
	pitch_speed=-pitch_speed;
	yaw_speed=-yaw_speed;
	
	if(pitch_speed!=0)
	{
		pitch_speedlast=pitch_speed;
	}
	yaw_speedlast=yaw_speed;
	
	uint8_t Data[8]={0};
	Data[0]=pitch_speedlast>>8;
	Data[1]=pitch_speedlast;
	Data[2]=yaw_speedlast>>8;
	Data[3]=yaw_speedlast;
	Data[4]=Data[5]=Data[7]=NULL;
	Data[6]=0x00;
	uint32_t t;
	CAN_Send_Msg(Data,8,t);
	//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,1);
}

void gimbal_send(int16_t yaw,int16_t pitch)
{
	uint8_t Data[8]={0};
	Data[0]=yaw>>8;
	Data[1]=yaw;
	Data[2]=pitch>>8;
	Data[3]=pitch;
	Data[4]=Data[5]=Data[7]=NULL;
	Data[6]=0x00;
	uint32_t t;
	CAN_Send_Msg(Data,8,t);
}

uint16_t Motor_CAN_Receive(void)
{
	int i;
//	CAN_Receive_Msg();
	
	if(CAN_Receive_Msg()==0)
	{  if(RxMsg.StdId==0x205)
		{
			for(i=0;i<8;i++)
			yaw[i]=buf[i];
		}
		if(RxMsg.StdId==0x206)
	  {
		for(i=0;i<8;i++)
			pitch[i]=buf[i];
	  }
		return 1;
	}
	return 0;
}

uint8_t yaw_count(void)
{

	yaw[0]=yaw[0]<<8;
	yaw[2]=yaw[2]<<8;
	yaw[4]=yaw[4]<<8;
	
	yaw_last[0]=yaw_counter[0];
	yaw_last[1]=yaw_counter[1];
	yaw_last[2]=yaw_counter[2];
	
	yaw_counter[0]=yaw[0]|yaw[1];
	yaw_counter[1] =yaw[2]|yaw[3];
	yaw_counter[2]=yaw[4]|yaw[5];
	
	
}
		
uint8_t pitch_count(void)
{
	pitch[0]=pitch[0]<<8;
	pitch[2]=pitch[2]<<8;
	pitch[4]=pitch[4]<<8;
	
	pitch_last[0]=pitch_counter[0];
	pitch_last[1]=pitch_counter[1];
	pitch_last[2]=pitch_counter[2];
	
	pitch_counter[0]=pitch[0]|pitch[1];
	pitch_counter[1]=pitch[2]|pitch[3];
	pitch_counter[2]=pitch[4]|pitch[5];
	
}
void GetEncoderBias(volatile Encoder *v, uint8_t *msg)
{

	v->ecd_bias = (msg[0] << 8) | msg[1]; //保存初始编码器值作为偏差
	v->ecd_value = v->ecd_bias;
	v->last_raw_value = v->ecd_bias;
	//v->temp_count++;
}
void EncoderProcess(volatile Encoder *v, uint8_t *msg)
{
	int i = 0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg[0] << 8) | msg[1];
	v->diff = v->raw_value - v->last_raw_value;
	if (v->diff < -6400) //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if (v->diff > 6400)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_relative=v->ecd_value*360/8192;
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias) * 360.0f / 8192 + v->round_cnt * 360;
//	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
//	if (v->buf_count == RATE_BUF_SIZE)
//	{
//		v->buf_count = 0;
//	}
//	//计算速度平均值
//	for (i = 0; i < RATE_BUF_SIZE; i++)
//	{
//		temp_sum += v->rate_buf[i];
//	}
//	v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}

void Motor_cailbration(void)
{
   uint8_t Data[8]={0};
	 Data[6]=0x04;
	CAN_Send_Msg(Data,8,0x1FF);
 }
//void control()
//{
//	
//	
//	if(pitch_abs==180)
//	{
//		Motor_CAN_Send(0,0);
//		HAL_Delay(10);
//		
//		//Motor_cailbration();
//		Motor_CAN_Send(1000,1000);
//	}
//}
 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	 
	 Motor_CAN_Receive();
	
	// __HAL_CAN_FIFO_RELEASE(hcan, CAN_RX_FIFO0);
	__HAL_CAN_ENABLE_IT(&hcan1,((uint32_t) CAN_IER_FMPIE0 ));
	// __HAL_CAN_DISABLE_IT(hcan,(uint32_t)CAN_IER_FOVIE0 | (uint32_t)CAN_IER_FMPIE0);
//	if(hcan->State == HAL_CAN_STATE_LISTENING )
//  {   
//    /* Disable Error warning, Error passive, Bus-off, Last error code
//       and Error Interrupts */
//    __HAL_CAN_DISABLE_IT(hcan, ((uint32_t)CAN_IER_EWGIE) |
//                              ((uint32_t)CAN_IER_EPVIE) |
//                               ((uint32_t)CAN_IER_BOFIE) |
//                               ((uint32_t)CAN_IER_LECIE) |
//                              ((uint32_t)CAN_IER_ERRIE));
//  }
//	yaw_count();
//	  pitch_count();
	//HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_FULL);
	EncoderProcess(&GMPitchEncoder,pitch);
   EncoderProcess(&GMYawEncoder,yaw);
	//control();
	
}

//void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
//{
//	Motor_CAN_Send(-pitch_speed,-yaw_speed);
//	HAL_Delay(100);
//}

 
void Motor_Init(void)
{
	//Motor_cailbration();
	//HAL_Delay(20000);
	//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14,1);
	//Motor_CAN_Send();
	//gimbal_send(1000,-1000);
	HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14);
	GetEncoderBias(&GMPitchEncoder,pitch);
//	HAL_Delay (100);
}
