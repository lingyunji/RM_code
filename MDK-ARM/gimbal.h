#include "stm32f4xx_hal.h"


typedef struct
{
    float Pitch;
    float Yaw;
}Gimbal_Ref_t;

//角度类型枚举
typedef enum
{
    AngleMode_REL,          //相对角度
    AngleMode_ABS,           //绝对角度
	AngleMode_ECD
}AngleMode_Enum;

typedef enum
{
    PID_ABS,			//直接设为输出
    PID_SPEED,			//Yaw轴数据导入速度环
	PID_POSITION		//导入位置环
}PIDMode_Enum;

typedef enum
{
	Gimbal_Prepare,
	Gimbal_Stop,
	Gimbal_RC_Mode,
	Gimbal_Mouse_Mode,
	Gimbal_Auto,//跟踪
	Gimbal_Standby,
	Gimbal_Debug,
}Gimbal_MoveMode_t;

typedef enum
{
	GMAutoPrepare,
	GMAutoReady,
	GMAutoSearch,
	GMAutoCatch,
	GMAutoShooting,
	GMAutoSlowDown,
	GMAutoCheck,
	GMAutoDefence,
}Gimbal_AutoMode_t;

typedef __packed struct
{
	float yaw;
	float pitch;
}CameraOffset_t;


void Gimbal_YawAngleSet(float Target, AngleMode_Enum mode);
void Gimbal_PitchAngleSet(float Target, AngleMode_Enum mode);
void GMCalLoop(PIDMode_Enum PIDMode);
void SetGimbalMotorOutput();

 
