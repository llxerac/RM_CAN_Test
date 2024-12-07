#ifndef _pid_
#define _pid_
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_conf.h"

typedef uint8_t u8;

typedef struct
{
	 //????PID??
	float Velcity_Kp;
	float Velcity_Ki;
	float Velcity_Kd;
	float Ur;				//???
	
	u8 PID_is_Enable;		//PID??
	int Un;					//?????
	int En_1;				//???????
	int En_2;				//???????
	int PWM;				//??PWM?
	
}PID_InitDefStruct;

void PID_Init(PID_InitDefStruct *p);
int Velocity_PID(int TargetVelocity,int CurrentVelocity,PID_InitDefStruct *p);
 
#endif
