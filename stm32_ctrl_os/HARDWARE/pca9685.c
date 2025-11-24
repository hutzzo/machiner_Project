#include "pca9685.h"
#include "I2C_PCA.h"
#include "delay.h"

static void wr(uint8_t reg, uint8_t val)
{
    PCA_I2C_WriteOneByte(PCA9685_ADDR, reg, val);
}

static uint8_t rd(uint8_t reg)
{
    return PCA_I2C_ReadOneByte(PCA9685_ADDR, reg);
}

void PCA9685_Init(void)
{
    wr(0x00, 0x00);
    wr(0x01, 0x04);
    PCA9685_SetPWMFreq(50);
}

void PCA9685_SetPWMFreq(uint16_t freq)
{
    float prescaleval;
    uint8_t prescale;
    uint8_t old;

    prescaleval = 25000000.0f;
    prescaleval /= 4096.0f;
    prescaleval /= (float)freq;
    prescaleval -= 1.0f;
    prescale = (uint8_t)(prescaleval + 0.5f);
    old = rd(0x00);
    wr(0x00, (old | 0x10));
    wr(0xFE, prescale);
    wr(0x00, (old & (uint8_t)(~0x10)));
    delay_ms(5);
    wr(0x00, ((old & (uint8_t)(~0x10)) | 0xA0));
}

void PCA9685_SetPWM(uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t reg;
    uint8_t buf[4];
    reg = (uint8_t)(0x06 + 4 * channel);
    buf[0] = (uint8_t)(on & 0xFF);
    buf[1] = (uint8_t)((on >> 8) & 0x0F);
    buf[2] = (uint8_t)(off & 0xFF);
    buf[3] = (uint8_t)((off >> 8) & 0x0F);
    PCA_I2C_WriteBuff(PCA9685_ADDR, reg, 4, buf);
}

void PCA9685_SetAllPWM(uint16_t on, uint16_t off)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(on & 0xFF);
    buf[1] = (uint8_t)((on >> 8) & 0x0F);
    buf[2] = (uint8_t)(off & 0xFF);
    buf[3] = (uint8_t)((off >> 8) & 0x0F);
    PCA_I2C_WriteBuff(PCA9685_ADDR, 0xFA, 4, buf);
}

void PCA9685_SetPWM_Duty(uint8_t channel, uint16_t duty)
{
    if (duty > 4095) duty = 4095;
    PCA9685_SetPWM(channel, 0, duty);
}