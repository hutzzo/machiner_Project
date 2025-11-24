#ifndef _I2C_PCA_H_
#define _I2C_PCA_H_
#include "system.h"
#include "sys.h"
#include "stdbool.h"
#include "stdint.h"

#define PCA_SDA_IN()  {GPIOC->MODER&=~(3<<(11*2));GPIOC->MODER|=0<<11*2;}
#define PCA_SDA_OUT() {GPIOC->MODER&=~(3<<(11*2));GPIOC->MODER|=1<<11*2;}
#define PCA_IIC_SCL    PCout(10)//引脚： PC10 为 SCL ， PC11 为 SDA
#define PCA_IIC_SDA    PCout(11)//引脚： PC10 为 SCL ， PC11 为 SDA
#define PCA_READ_SDA   PCin(11)
void PCA_I2C_GPIOInit(void);
void PCA_I2C_Start(void);
void PCA_I2C_Stop(void);
bool PCA_I2C_WaiteForAck(void);
void PCA_I2C_Ack(void);
void PCA_I2C_NAck(void);
void PCA_I2C_WriteByte(uint8_t Data);
uint8_t PCA_I2C_ReadByte(uint8_t Ack);
uint8_t PCA_I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t PCA_I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
bool PCA_I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
bool PCA_I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
#endif
