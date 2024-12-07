#include "pid.h"
//#include "PWM_Config.h"
#include "usart.h"   //USART??
 
void PID_Init(PID_InitDefStruct *p)
{
	p->Velcity_Kp = 1.8;
	p->Velcity_Ki = 0;
	p->Velcity_Kd = 0.2;
	p->Ur = 2000;
	p->PID_is_Enable = 1;
	p->Un = 0;
	p->En_1 = 0;			//误差累计
	p->En_2 = 0;			//上一次误差
	p->PWM = 300;

}

int Velocity_PID(int TargetVelocity,int CurrentVelocity,PID_InitDefStruct *p)
{
	if(p->PID_is_Enable == 1)
	{
		int En = TargetVelocity - CurrentVelocity;//???                                                     
	
//		p->Un += p->Velcity_Kp*(En - p->En_1) + p->Velcity_Ki*En + p->Velcity_Kd*(En - 2*p->En_1 + p->En_2);//???PID
//		//printf("kp:%.1f, ki:%.1f, kd:%.1f\r\n", p->Velcity_Kp, p->Velcity_Ki, p->Velcity_Kd);
//		
//		p->En_2=p->En_1;
//		p->En_1=En;
//		printf("Un:%d, En1:%d, En2:%d\r\n", p->Un, p->En_1, p->En_2);
//		p->PWM += p->Un;
		p->En_1 += En;
		p->Un = p->Velcity_Kp * En + p->Velcity_Ki * p->En_1 + p->Velcity_Kd * (En - p->En_2);
		p->En_2 = En;
		p->PWM += p->Un;
		/*????*/
		if(p->PWM>p->Ur) p->PWM=p->Ur;
		if(p->PWM<-p->Ur) p->PWM=-p->Ur;
		return p->PWM;
	}
	else
	{
		PID_Init(p);
		printf("初始化\r\n");
	}
	
}
