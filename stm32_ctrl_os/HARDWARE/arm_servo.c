#include "arm_servo.h"
#include "pca9685.h"  // 机械臂舵机使用PCA9685控制

static int curr[6] = {1500,1500,1500,1500,1500,1000};
static int target[6] = {1500,1500,1500,1500,1500,1000};
static int step = 25;
static int smooth_band = 0;

static int step_limit(int a,int b){
 int d=b-a;
 int ad = d>0?d:-d;
 if(ad>smooth_band) return b;
 if(d>step) return a+step;
 if(d<-step) return a-step;
 return b;
}

void ArmServo_Init(void)
{
 uint16_t duty;
 int i;
 
 // 初始化当前值和目标值
 curr[0]=1500; curr[1]=1500; curr[2]=1500; curr[3]=1500; curr[4]=1500; curr[5]=1000;
 target[0]=curr[0]; target[1]=curr[1]; target[2]=curr[2]; target[3]=curr[3]; target[4]=curr[4]; target[5]=curr[5];
 
 // 机械臂舵机通过PCA9685输出（不使用TIM8/TIM12，避免与转向舵机冲突）
 // PCA9685初始化已在system.c中完成，这里只设置初始PWM
 for(i=0; i<6; i++) {
     duty = (uint16_t)((curr[i] * 4096) / 20000);  // PWM微秒转PCA9685计数值
     PCA9685_SetPWM(i, 0, duty);
 }
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
 uint16_t duty;
 int i;
 
 // 平滑过渡到目标值
 curr[0]=step_limit(curr[0],target[0]);
 curr[1]=step_limit(curr[1],target[1]);
 curr[2]=step_limit(curr[2],target[2]);
 curr[3]=step_limit(curr[3],target[3]);
 curr[4]=step_limit(curr[4],target[4]);
 curr[5]=step_limit(curr[5],target[5]);
 
 // 输出到PCA9685（机械臂舵机专用）
 for(i=0; i<6; i++) {
     duty = (uint16_t)((curr[i] * 4096) / 20000);  // PWM微秒转PCA9685计数值
     PCA9685_SetPWM(i, 0, duty);
 }
}
