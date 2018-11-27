#include "angle.h"
#include "math.h"
#define M_E 2.718281828f

Smooth_t GMYawSmooth=SMOOTH_GMINIT;
Smooth_t GMPitchSmooth=SMOOTH_GMINIT;
uint16_t test_counter = 0;


/**
  * @note   Optimize
  * @brief  在有导电滑环的情况下最优化相对角度计算，
  * @brief  计算出达到目标角度的最小偏转角度
  * @param  float angle_now:此刻角度
  * @param  float angle_ref:目标角度
  * @retval float 计算出的角度，范围在-180.0 ~ +180.0
  * @note   
  */
float Angle_Optimize(float angle_now, float angle_ref)
{
  float temp_now, temp_ref;
  temp_now = angle_now - (int)(angle_now / 360) * 360;
  temp_ref = angle_ref - (int)(angle_ref / 360) * 360;
  if (temp_ref - temp_now > 180)
    return (temp_ref - temp_now - 360.0f);
  else if (temp_ref - temp_now < -180)
    return (temp_ref - temp_now + 360.0f);
  else
    return (temp_ref - temp_now);
}
/**
  * @note   modified
  * @brief  SIGMOID平滑函数  y = 1/(1+z^(-t))
  * @param  Angle_Ref 目标参数
  * @param  Angle_Last 起始参数
  * @param  平滑过程长度(次数)
  * @retval 平滑后的角度
  */
float SmoothSigmoid(float Angle_Ref, int16_t smooth_time, Smooth_t *obj)
{
					test_counter++;
	if (obj->SmoothMode == SMOOTH_SINGLE)
	{
		if (obj->Updata_Flag == 1) //单词平滑初始化置位
		{
			obj->Angle_Last = obj->Angle_Mark;
			obj->Angle_Mark = Angle_Ref;
			obj->t = -5;
			obj->Updata_Flag = 0;
		}
	}
	else
	{
		if (Angle_Ref != obj->Angle_Mark) //角度发生变化 进行平滑
		{
			obj->Angle_Last = obj->Angle_Mark;
			obj->Angle_Mark = Angle_Ref;
			obj->t = -5;
		}
	}
	float output = obj->Angle_Last + (Angle_Ref - obj->Angle_Last) * (1.0f / (1 + pow(M_E, -(obj->t))));
	if (obj->t > 5)
		output = Angle_Ref;
	else
		obj->t += (10.0f / smooth_time);
	return output;
}
/**
  * @brief  角度换算，换算成-180~180
  * @param  float angle:待换算角度
  * @retval  float angle:换算后角度
  * @note   
  */
float Angle_Process(float angle)
{
	angle=angle-360*(int16_t)(angle/360);
    if(angle>180)
    	angle-=360;
	return angle;
}
