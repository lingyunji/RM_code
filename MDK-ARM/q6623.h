#include "stm32f4xx_hal.h"

typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;	//经过处理后连续的编码器值
	int32_t ecd_relative;
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	//int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;											//角度
	float ecd_distance;						//积分距离（单位cm）
}Encoder;


extern CAN_HandleTypeDef hcan1;
extern float pitch_abs;
extern float yaw_abs;
extern int32_t pitch_diff;
extern int32_t yaw_diff;
extern int32_t pitch_relative;
extern int32_t yaw_relative;
extern volatile Encoder GMYawEncoder ;
extern volatile Encoder GMPitchEncoder ;


void Motor_CAN_Send();
uint16_t Motor_CAN_Receive(void);
uint8_t yaw_count(void);
uint8_t pitch_count(void);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef* hcan);
void Motor_cailbration(void);
void Motor_Init(void);
void control(void);
void gimbal_send(int16_t pitch,int16_t yaw);
void GetEncoderBias(volatile Encoder *v, uint8_t *msg);