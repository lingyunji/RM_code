#include "gimbal.h"
#include "PID.h"
#include "imu.h"
#include "q6623.h"

#define offset_pitch -2

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;
Gimbal_Ref_t GimbalRef;
extern imu_t imu;

/**
  * @note   modified
  * @brief  云台Yaw角度设置
  * @param  目标角度
  * @param  模式 AngleMode_REL(相对当前位置 每次调用 都会根据当前角度和输入参数来改变参考角度)     AngleMode_ABS     AngleMode_ECD 编码器角度
  * @retval void
  */
void Gimbal_YawAngleSet(float Target, AngleMode_Enum mode)
{
//#if (XDRM_CONVERTER == 0)
	if (mode == AngleMode_REL)
	{
		Target += yaw_angle;
	}
	if (mode == AngleMode_ECD)
	{
		
		Target += yaw_angle;
		Target += GMYawEncoder.ecd_angle;
	}
//#else
//	if (mode == AngleMode_REL)
//	{
//		Target += IMU9250_Data.Angle_Yaw;
//	}
//	if (mode == AngleMode_ECD)
//	{
//		Target += IMU9250_Data.Angle_Yaw;
//		Target += GMYawEncoder.ecd_angle;
//	}
//#endif
	GimbalRef.Yaw = Target;
}
/**
  * @note   modified
  * @brief  云台Pitch角度及模式设置
  * @param  目标角度(绝对角度）
  * @retval void
  * @note   Pitch
  */
void Gimbal_PitchAngleSet(float Target, AngleMode_Enum mode)
{
	if (mode == AngleMode_REL)
	{
		
		Target += GMPitchEncoder.ecd_angle;
	}
	GimbalRef.Pitch = Target;
}
/**
  * @note   modified
  * @brief  云台PID计算
  * @param  void
  * @retval void
  * @note   
  */
void GMCalLoop(PIDMode_Enum PIDMode)
{
	if (PIDMode==PID_ABS)
	{
		GMYSpeedPID.output = GimbalRef.Yaw;
		GMPSpeedPID.output = GimbalRef.Pitch;
	}
	else if(PIDMode==PID_SPEED)
	{
		PID_Task(&GMYSpeedPID, GimbalRef.Yaw, MPU6050_Real_Data.Gyro_Z);
		PID_Task(&GMPPositionPID, GimbalRef.Pitch,GMPitchEncoder.ecd_angle );
		PID_Task(&GMPSpeedPID, GMPPositionPID.output, MPU6050_Real_Data.Gyro_Y);
	}
	else
	{
	//	#if (XDRM_CONVERTER == 0)
		PID_Task(&GMYPositionPID, GimbalRef.Yaw, yaw_angle);
		PID_Task(&GMYSpeedPID, GMYPositionPID.output, MPU6050_Real_Data.Gyro_Z);

		PID_Task(&GMPPositionPID, GimbalRef.Pitch, GMPitchEncoder.ecd_angle);
		PID_Task(&GMPSpeedPID, GMPPositionPID.output, MPU6050_Real_Data.Gyro_Y);
//		PID_Task(&GMYPositionPID, GimbalRef.Yaw, yaw_angle);
//		PID_Task(&GMYSpeedPID, GMYPositionPID.output, MPU6050_Real_Data.Gyro_Z);

//		PID_Task(&GMPPositionPID, GimbalRef.Pitch, GMPitchEncoder.ecd_angle);
//		PID_Task(&GMPSpeedPID, GMPPositionPID.output, MPU6050_Real_Data.Gyro_X);
		
//		#else
//		PID_Task(&GMYPositionPID, GimbalRef.Yaw, IMU9250_Data.Angle_Yaw);
//		PID_Task(&GMYSpeedPID, GMYPositionPID.output, IMU9250_Data.Gyro_Yaw);

//		PID_Task(&GMPPositionPID, GimbalRef.Pitch, GMPitchEncoder.ecd_angle);
//		PID_Task(&GMPSpeedPID, GMPPositionPID.output, IMU9250_Data.Gyro_Roll);
//		#endif
	}
}

/**
  * @brief  使用上位机给的PID值
  * @param  None
  * @retval None
  */
//void GM_PID_Calibration(void)
//{
//	GMYPositionPID.kp = AppParamRealUsed.YawPositionPID.kp_offset / 10.0;
//	GMYPositionPID.ki = AppParamRealUsed.YawPositionPID.ki_offset / 1000.0;
//	GMYPositionPID.kd = AppParamRealUsed.YawPositionPID.kd_offset / 10.0;

//	GMYSpeedPID.kp = AppParamRealUsed.YawSpeedPID.kp_offset / 10.0;
//	GMYSpeedPID.ki = AppParamRealUsed.YawSpeedPID.ki_offset / 1000.0;
//	GMYSpeedPID.kd = AppParamRealUsed.YawSpeedPID.kd_offset / 10.0;

//	GMPPositionPID.kp = AppParamRealUsed.PitchPositionPID.kp_offset / 10.0;
//	GMPPositionPID.ki = AppParamRealUsed.PitchPositionPID.ki_offset / 1000.0;
//	GMPPositionPID.kd = AppParamRealUsed.PitchPositionPID.kd_offset / 10.0;

//	GMPSpeedPID.kp = AppParamRealUsed.PitchSpeedPID.kp_offset / 10.0;
//	GMPSpeedPID.ki = AppParamRealUsed.PitchSpeedPID.ki_offset / 1000.0;
//	GMPSpeedPID.kd = AppParamRealUsed.PitchSpeedPID.kd_offset / 10.0;
//}

/**
  * @note   modified
  * @brief  电流值通过CAN发送给电机
  * @param  void
  * @retval void
  * @note   
  */
void SetGimbalMotorOutput(void)
{
	//云台控制输出
//	if (GMPitchEncoder.ecd_value==90)
//	{
//		gimbal_send( 0, 0); //yaw + pitch
//		HAL_Delay(1000);
//	}
//	else
//	{
	//gimbal_send(0,0);
	//	gimbal_send( (int16_t)GMYSpeedPID.output, -(int16_t)GMPSpeedPID.output); //
	gimbal_send( 0, -(int16_t)GMPSpeedPID.output);
//	}
}