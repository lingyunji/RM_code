#include "PID.h"

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

float tX[3];
float k1=4.0f;
float k2=9.0f;
float k3=1.0f;
float k4=3.0f;
float tY[3];

void PID_Reset(PID_Regulator_t *pid)
{
	pid->err[0]=0;
	pid->err[1]=0;
	pid->err[2]=0;
	pid->output=0;
	tX[2]=tX[1]=tX[0]=tY[0]=tY[2]=tY[1]=0;
}

void PID_TEST_YP(PID_Regulator_t *pid)
{
	tY[2]=tY[1];
	tY[1]=tY[0];
	tY[0]=pid->ref;
	
	pid->err[0]=pid->err[1];
	pid->err[1]=pid->ref-pid->fdb;//误差
	pid->err[2]+=pid->err[1];//积分
	VAL_LIMIT(pid->err[2],-pid->componentKiMax/pid->ki,pid->componentKiMax/pid->ki);//抗饱和积分
	pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0])+k3*(tY[0]-tY[1]);
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}

void PID_TEST_YS(PID_Regulator_t *pid)
{
	pid->err[0]=pid->err[1];
	pid->err[1]=pid->ref-pid->fdb;
	pid->err[2]+=pid->err[1];
	VAL_LIMIT(pid->err[2],-pid->componentKiMax/pid->ki,pid->componentKiMax/pid->ki);
	pid->output=pid->kp*pid->err[1]+pid->ki*pid->err[2]+pid->kd*(pid->err[1]-pid->err[0])+k3*(tY[0]-tY[1]);
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}

void PID_TEST_P(PID_Regulator_t *pid)
{
	tX[2]=tX[1];
	tX[1]=tX[0];
	tX[0]=pid->ref;
	
	pid->err[0]=pid->err[1];
	pid->err[1]=pid->ref-pid->fdb;
	pid->err[2]+=pid->err[1];
	VAL_LIMIT(pid->err[2],-pid->componentKiMax/pid->ki,pid->componentKiMax/pid->ki);	//抗饱和积分
	pid->output=pid->kp*pid->err[1]  +  pid->ki*pid->err[2]  +    pid->kd*(pid->err[1]-pid->err[0])+k1*(tX[0]-tX[1]);	
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}

void PID_TEST_S(PID_Regulator_t *pid)
{
	pid->err[0]=pid->err[1];
	pid->err[1]=pid->ref-pid->fdb;//误差
	pid->err[2]+=pid->err[1];//积分
	VAL_LIMIT(pid->err[2],-pid->componentKiMax/pid->ki,pid->componentKiMax/pid->ki);	//抗饱和积分
	pid->output=pid->kp*pid->err[1]  +  pid->ki*pid->err[2]  +    pid->kd*(pid->err[1]-pid->err[0])+k2*(tX[0]-2*tX[1]+tX[2]);	
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}

void PID_Calc(PID_Regulator_t *pid)
{
	pid->err[0]=pid->err[1];
	pid->err[1]=pid->ref-pid->fdb;//误差
	pid->err[2]+=pid->err[1];//积分
	VAL_LIMIT(pid->err[2],-pid->componentKiMax/pid->ki,pid->componentKiMax/pid->ki);	//抗饱和积分
	pid->output=pid->kp*pid->err[1]  +  pid->ki*pid->err[2]  +    pid->kd*(pid->err[1]-pid->err[0]);	
	VAL_LIMIT(pid->output,-pid->outputMax,pid->outputMax);
}

/**
	* @brief PID计算
	* @param PID_Regulator_t *PID_Stucture
	* @param float ref
	* @param float fdb
	* @retval float output
*/
float PID_Task(PID_Regulator_t *PID_Stucture, float ref, float fdb)
{
	PID_Stucture->ref = ref;
	PID_Stucture->fdb = fdb;
	PID_Stucture->Calc(PID_Stucture);
	return PID_Stucture->output;
}

float PID_Task_test(PID_Regulator_t *PID_Stucture, float ref, float fdb)
{
	PID_Stucture->ref = ref;
	PID_Stucture->fdb = fdb;

	PID_Stucture->err[0]=PID_Stucture->err[1];
	PID_Stucture->err[1]=PID_Stucture->ref-PID_Stucture->fdb;
	PID_Stucture->err[2]+=PID_Stucture->err[1];
	VAL_LIMIT(PID_Stucture->err[2],-PID_Stucture->componentKiMax/PID_Stucture->ki,PID_Stucture->componentKiMax/PID_Stucture->ki);	//抗饱和积分
	PID_Stucture->output=PID_Stucture->kp*PID_Stucture->err[1]+PID_Stucture->ki*PID_Stucture->err[2]+PID_Stucture->kd*(PID_Stucture->err[1]-PID_Stucture->err[0]);	
	return PID_Stucture->output;
}
