#ifndef __SYSTEM_H
#define __SYSTEM_H

// Refer to all header files you need
//����������Ҫ�õ���ͷ�ļ�
#include "FreeRTOSConfig.h"
//FreeRTOS���ͷ�ļ� 
//FreeRTOS related header files
#include "FreeRTOS.h"
#include "stm32f4xx.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
//The associated header file for the peripheral 
//��������ͷ�ļ�
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "show.h"								   
#include "pstwo.h"
#include "key.h"
#include "robot_select_init.h"
#include "I2C.h"
#include "I2C_PCA.h"
#include "pca9685.h"
#include "MPU6050.h"
#include "F4_flash.h"

#include "ICM20948.h"
#include "imu_task.h"

#include "usb_core.h"
#include "usbh_core.h"
#include "usbh_hid_core.h"
#include "usbh_usr.h"

extern USBH_HOST  USB_Host;
extern USB_OTG_CORE_HANDLE  USB_OTG_Core_dev;

// Enumeration of car types
//С���ͺŵ�ö�ٶ���
typedef enum 
{
    Mec_Car = 0, 
    FourWheel_Car, 
    Tank_Car,
    Akm_Car
} CarMode;

//Motor speed control related parameters of the structure
//����ٶȿ�����ز����ṹ��
typedef struct  
{
	float Encoder;     //Read the real time speed of the motor by encoder //��������ֵ����ȡ���ʵʱ�ٶ�
	float Motor_Pwm;   //Motor PWM value, control the real-time speed of the motor //���PWM��ֵ�����Ƶ��ʵʱ�ٶ�
	float Target;      //Control the target speed of the motor //���Ŀ���ٶ�ֵ�����Ƶ��Ŀ���ٶ�
	float Velocity_KP; //Speed control PID parameters //�ٶȿ���PID����
	float	Velocity_KI; //Speed control PID parameters //�ٶȿ���PID����
}Motor_parameter;

//Smoothed the speed of the three axes
//ƽ��������������ٶ�
typedef struct  
{
	float VX;
	float VY;
	float VZ;
}Smooth_Control;

/****** external variable definition. When system.h is referenced in other C files, 
        other C files can also use the variable defined by system.c           ******/
/****** �ⲿ�������壬������c�ļ�����system.hʱ��Ҳ����ʹ��system.c����ı��� ******/
extern u8 Flag_Stop;
extern int Divisor_Mode;
extern u8 Car_Mode;
extern int Servo;
extern float RC_Velocity;
extern float Move_X, Move_Y, Move_Z; 
extern float Velocity_KP, Velocity_KI;	
extern Smooth_Control smooth_control;
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D; 
extern float Steering_Angle;
extern u8 Steering_Manual_Flag;
extern u16 Moveit_Active_Counter;
extern float Encoder_precision;
extern float Wheel_perimeter;
extern float Wheel_spacing; 
extern float Axle_spacing; 
extern float Omni_turn_radiaus; 
extern u8 PS2_ON_Flag, APP_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart1_ON_Flag, Usart5_ON_Flag;
extern u8 Flag_Left, Flag_Right, Flag_Direction, Turn_Flag; 
extern u8 PID_Send;                                            										                 
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;
extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern long int ErrorCode; 
extern float Moveit_Angle1,Moveit_Angle2,Moveit_Angle3,Moveit_Angle4,Moveit_Angle5,Moveit_Angle6;
extern float Moveit_Target_Angle1,Moveit_Target_Angle2,Moveit_Target_Angle3,Moveit_Target_Angle4,Moveit_Target_Angle5,Moveit_Target_Angle6;
extern float Moveit_Max_Speed;
extern int  Moveit_PWM1,Moveit_PWM2,Moveit_PWM3,Moveit_PWM4,Moveit_PWM5,Moveit_PWM6;

extern u8 Proc_Flag;
extern int servo_flag;
extern int Servo_Count[6];
extern int check_time_count_motor_forward,check_time_count_motor_retreat;
extern int POT_val;
extern int Arm_direction;
extern int TIM8_Servo_flag;
extern u8 uart3_receive_message[50];
extern u8 uart3_send_flag;
extern u8 message_count;
extern u8 uart2_receive_message[50];
extern u8 uart2_send_flag;
extern u8 app_count;
extern int Full_rotation;

void systemInit(void);

/***Macros define***/ /***�궨��***/
//After starting the car (1000/100Hz =10) for seconds, it is allowed to control the car to move
//����(1000/100hz=10)������������С�������˶�
#define CONTROL_DELAY		1000
//The number of robot types to determine the value of Divisor_Mode. There are currently 6 car types
//�������ͺ�����������Divisor_Mode��ֵ��Ŀǰ��6��С������
#define CAR_NUMBER    4      
#define RATE_1_HZ		  1
#define RATE_5_HZ		  5
#define RATE_10_HZ		10
#define RATE_20_HZ		20
#define RATE_25_HZ		25
#define RATE_50_HZ		50
#define RATE_100_HZ		100
#define RATE_200_HZ 	200
#define RATE_250_HZ 	250
#define RATE_500_HZ 	500
#define RATE_1000_HZ 	1000
/***Macros define***/ /***�궨��***/

//C library function related header file
//C�⺯�������ͷ�ļ�
#include <stdio.h> 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif
