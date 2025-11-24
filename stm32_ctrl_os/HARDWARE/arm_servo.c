#include "arm_servo.h"

static int curr[6] = {1500,1500,1500,1500,1500,1000};
static int target[6] = {1500,1500,1500,1500,1500,1000};
static int step = 10;

static int step_limit(int a,int b){
 int d=b-a;
 if(d>step) return a+step;
 if(d<-step) return a-step;
 return b;
}

void ArmServo_Init(void)
{
 curr[0]=1500; curr[1]=1500; curr[2]=1500; curr[3]=1500; curr[4]=1500; curr[5]=1000;
 target[0]=curr[0]; target[1]=curr[1]; target[2]=curr[2]; target[3]=curr[3]; target[4]=curr[4]; target[5]=curr[5];
 TIM8->CCR1=curr[0];
 TIM8->CCR2=curr[1];
 TIM8->CCR3=curr[2];
 TIM8->CCR4=curr[3];
 TIM12->CCR1=curr[4];
 TIM12->CCR2=curr[5];
}

void ArmServo_SetTargets(int s1,int s2,int s3,int s4,int s5,int s6)
{
 target[0]=s1;
 target[1]=s2;
 target[2]=s3;
 target[3]=s4;
 target[4]=s5;
 target[5]=s6;
}

void ArmServo_Update(void)
{
 curr[0]=step_limit(curr[0],target[0]);
 curr[1]=step_limit(curr[1],target[1]);
 curr[2]=step_limit(curr[2],target[2]);
 curr[3]=step_limit(curr[3],target[3]);
 curr[4]=step_limit(curr[4],target[4]);
 curr[5]=step_limit(curr[5],target[5]);
 TIM8->CCR1=curr[0];
 TIM8->CCR2=curr[1];
 TIM8->CCR3=curr[2];
 TIM8->CCR4=curr[3];
 TIM12->CCR1=curr[4];
 TIM12->CCR2=curr[5];
}
