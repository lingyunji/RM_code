#include "stm32f4xx_hal.h"

#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f		
#define STICK_TO_YAW_ANGLE_INC_FACT         0.004f


void Gimbal_control();
void Ref_UpdataFromRCStick(void);