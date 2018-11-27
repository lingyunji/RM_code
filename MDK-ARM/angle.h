#include "stm32f4xx_hal.h"

#define SMOOTH_CMINIT {0,0,0,0,SMOOTH_ACTIVE}
#define SMOOTH_GMINIT {0,0,0,0,SMOOTH_SINGLE}


typedef enum
{
	SMOOTH_SINGLE,
	SMOOTH_ACTIVE,
}SmoothMode_e;

typedef struct
{
	float Angle_Mark;
	float Angle_Last;
	float t;
	uint8_t Updata_Flag;
	SmoothMode_e SmoothMode;
}Smooth_t;


extern float Angle_Optimize(float angle_now, float angle_ref);
extern float SmoothSigmoid(float Angle_Ref, int16_t smooth_time, Smooth_t *obj);
extern float Angle_Process(float angle);