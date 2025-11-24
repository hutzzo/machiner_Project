#include "I2C_PCA.h"
#include "delay.h"
void PCA_I2C_GPIOInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;//引脚： PC10 为 SCL ， PC11 为 SDA
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    PCA_IIC_SCL=1;PCA_IIC_SDA=1;
}
void PCA_I2C_Start(void){PCA_SDA_OUT();PCA_IIC_SDA=1;PCA_IIC_SCL=1;delay_us(2);PCA_IIC_SDA=0;delay_us(2);PCA_IIC_SCL=0;}
void PCA_I2C_Stop(void){PCA_SDA_OUT();PCA_IIC_SCL=0;PCA_IIC_SDA=0;delay_us(1);PCA_IIC_SCL=1;PCA_IIC_SDA=1;delay_us(1);} 
bool PCA_I2C_WaiteForAck(void){u8 t=0;PCA_SDA_IN();PCA_IIC_SDA=1;delay_us(1);PCA_IIC_SCL=1;delay_us(1);while(PCA_READ_SDA){t++;if(t>50){PCA_I2C_Stop();return false;}delay_us(1);}PCA_IIC_SCL=0;return true;}
void PCA_I2C_Ack(void){PCA_IIC_SCL=0;PCA_SDA_OUT();PCA_IIC_SDA=0;delay_us(1);PCA_IIC_SCL=1;delay_us(1);PCA_IIC_SCL=0;}
void PCA_I2C_NAck(void){PCA_IIC_SCL=0;PCA_SDA_OUT();PCA_IIC_SDA=1;delay_us(1);PCA_IIC_SCL=1;delay_us(1);PCA_IIC_SCL=0;}
void PCA_I2C_WriteByte(uint8_t Data){u8 t;PCA_SDA_OUT();PCA_IIC_SCL=0;for(t=0;t<8;t++){PCA_IIC_SDA=(Data&0x80)>>7;Data<<=1;delay_us(1);PCA_IIC_SCL=1;delay_us(1);PCA_IIC_SCL=0;delay_us(1);}}
uint8_t PCA_I2C_ReadByte(uint8_t Ack){uint8_t i, d=0;PCA_SDA_IN();for(i=0;i<8;i++){PCA_IIC_SCL=0;delay_us(1);PCA_IIC_SCL=1;d<<=1;if(PCA_READ_SDA)d|=0x01;else d&=~0x01;delay_us(1);}if(Ack==0)PCA_I2C_NAck();else PCA_I2C_Ack();return d;}
uint8_t PCA_I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data){PCA_I2C_Start();PCA_I2C_WriteByte(DevAddr);PCA_I2C_WaiteForAck();PCA_I2C_WriteByte(RegAddr);PCA_I2C_WaiteForAck();PCA_I2C_WriteByte(Data);PCA_I2C_WaiteForAck();PCA_I2C_Stop();return 1;}
uint8_t PCA_I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr){uint8_t v;PCA_I2C_Start();PCA_I2C_WriteByte(DevAddr);PCA_I2C_WaiteForAck();PCA_I2C_WriteByte(RegAddr);PCA_I2C_WaiteForAck();PCA_I2C_Start();PCA_I2C_WriteByte(DevAddr|0x01);PCA_I2C_WaiteForAck();v=PCA_I2C_ReadByte(0);PCA_I2C_Stop();return v;}
bool PCA_I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff){uint8_t i;if(Num==0||pBuff==NULL)return false;PCA_I2C_Start();PCA_I2C_WriteByte(DevAddr);PCA_I2C_WaiteForAck();PCA_I2C_WriteByte(RegAddr);PCA_I2C_WaiteForAck();for(i=0;i<Num;i++){PCA_I2C_WriteByte(*(pBuff+i));PCA_I2C_WaiteForAck();}PCA_I2C_Stop();return true;}
bool PCA_I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff){uint8_t i;if(Num==0||pBuff==NULL)return false;PCA_I2C_Start();PCA_I2C_WriteByte(DevAddr);PCA_I2C_WaiteForAck();PCA_I2C_WriteByte(RegAddr);PCA_I2C_WaiteForAck();PCA_I2C_Start();PCA_I2C_WriteByte(DevAddr|0x01);PCA_I2C_WaiteForAck();for(i=0;i<Num;i++){if(i==Num-1)pBuff[i]=PCA_I2C_ReadByte(0);else pBuff[i]=PCA_I2C_ReadByte(1);}PCA_I2C_Stop();return true;}