#ifndef __ARM_SERVO_H
#define __ARM_SERVO_H
#include "system.h"
void ArmServo_Init(void);
void ArmServo_SetTargets(int s1,int s2,int s3,int s4,int s5,int s6);
void ArmServo_Update(void);
#endif
