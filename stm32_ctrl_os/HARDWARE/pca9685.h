#ifndef _PCA9685_H_
#define _PCA9685_H_
#include "stdint.h"
#define PCA9685_ADDR 0x80
void PCA9685_Init(void);
void PCA9685_SetPWMFreq(uint16_t freq);
void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off);
void PCA9685_SetAllPWM(uint16_t on, uint16_t off);
void PCA9685_SetPWM_Duty(uint8_t channel, uint16_t duty);
#endif
