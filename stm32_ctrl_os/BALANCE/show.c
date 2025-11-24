#include "show.h"
int Voltage_Show;
unsigned char i;          
unsigned char Send_Count; 
extern SEND_DATA Send_Data;
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; 
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;
/**************************************************************************
Function: Read the battery voltage, buzzer alarm, start the self-test, send data to APP, OLED display task
Input   : none
Output  : none
�������ܣ���ȡ��ص�ѹ�������������������Լ졢��APP�������ݡ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
int Buzzer_count=25;
u8 oled_refresh_flag;
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
   {	
		int i=0;
		static int LowVoltage_1=0, LowVoltage_2=0;
		static int Servo_adjust_timecount;
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_10_HZ));//This task runs at 10Hz //��������10Hz��Ƶ������
		
		//����ʱ���������ݷ�������������
		//The buzzer will beep briefly when the machine is switched on
		if(SysVal.Time_count<50)Buzzer=1; 
		else if(SysVal.Time_count>=51 && SysVal.Time_count<100)Buzzer=0;
		 
		if(LowVoltage_1==1 || LowVoltage_2==1)Buzzer_count=0;
		if(Buzzer_count<5)Buzzer_count++;
		if(Buzzer_count<5)Buzzer=1; //The buzzer is buzzing //����������
		else if(Buzzer_count==5)Buzzer=0;
		
		if(Beep_Control_Flag)
		{
			Buzzer = 1;
			Servo_adjust_timecount++;
			if(Servo_adjust_timecount == 10)
			{
				Beep_Control_Flag = 0;
				Buzzer = 0;
				Servo_adjust_timecount = 0;
				OLED_Refresh_Gram(); //ˢ����Ļ
        OLED_Clear();  //�����Ļ
		    OLED_Refresh_Gram();
			}
		}
		 
		//Read the battery voltage //��ȡ��ص�ѹ
		for(i=0;i<10;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/10;
		Voltage_All=0;
		
		if(LowVoltage_1==1)LowVoltage_1++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(LowVoltage_2==1)LowVoltage_2++; //Make sure the buzzer only rings for 0.5 seconds //ȷ��������ֻ��0.5��
		if(Voltage>=12.6f)Voltage=12.6f;
		else if(10<=Voltage && Voltage<10.5f && LowVoltage_1<2)LowVoltage_1++; //10.5V, first buzzer when low battery //10.5V���͵���ʱ��������һ�α���
		else if(Voltage<10 && LowVoltage_1<2)LowVoltage_2++; //10V, when the car is not allowed to control, the buzzer will alarm the second time //10V��С����ֹ����ʱ�������ڶ��α���
					
		APP_Show();	 //Send data to the APP //��APP��������
		if(oled_refresh_flag) OLED_Clear(),oled_refresh_flag=0;
		else oled_show(); //Tasks are displayed on the screen //��ʾ����ʾ����
   }
}  

/**************************************************************************
Function: The OLED display displays tasks
Input   : none
Output  : none
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{  
	///////////// usb ps2 �豸�����ʾ /////////////
	static u8 clear=0;
	static u8 show_done=0;
	
	static int count=0;	 
	 int Car_Mode_Show;
	
    Car_Mode_Show = Car_Mode;
		 Voltage_Show=Voltage*100; 
		 count++;
	memset(OLED_GRAM,0, 128*8*sizeof(u8));	//GRAM���㵫������ˢ�£���ֹ����
	
	if(Check==0)
	{
	if(usb_wait_EnumReady==EnumWait)
	{
		if(clear) 
		{
			clear=0,oled_refresh_flag=1; //ö����,������ʾ������Ϣ
			return;//ִ��1������
		}
		OLED_DrawBMP(32,1,96,7,gImage_usb_bmp);
		OLED_ShowString(12,50,"USB Init..");
		OLED_ShowNumber(85,50,enum_error,2,12);//ö�ٴ���Ĵ���
		OLED_Refresh_Line();
		show_done = 1;
		return;
	}
	else if( usb_wait_EnumReady==EnumDone )
	{
		if(show_done) //ö�ٳɹ�,��ʱ��ʾ1����ʾ����Ϣ
		{
			static u8 show_delay=0;
			if(++show_delay<RATE_10_HZ)
			{
				OLED_ShowString(12,50,"USB Init OK   ");
				OLED_Refresh_Line();
				return;
			}
			show_done = 0;
			show_delay=0;	
			oled_refresh_flag = 1;//�ӳ����,������ʾ��������
		}
		clear=1; //ö�����,�ȴ��´�ö��ʱ��������
	}
	///////////// usb ps2 �豸�����ʾ /////////////
   
	
	 //Collect the tap information of the potentiometer, 
	 //and display the car model to be fitted when the car starts up in real time
	 //�ɼ���λ����λ��Ϣ��ʵʱ��ʾС������ʱҪ�����С���ͺ�
	if(Servo_init_angle_adjust == 0)//�������ƣ�������΢��ģʽ
	{

		
		 //if(Check==0)//The car displays normally when the self-check mode is not enabled //û�п����Լ�ģʽʱС��������ʾ
		 {	
			 //The first line of the display displays the content//
			 //��ʾ����1����ʾ����//
             switch(Car_Mode_Show)
             {
                case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; 
                case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; 
                case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; 
                case Akm_Car:       OLED_ShowString(0,0,"AKM "); break; 
             }
			 
			 if(Car_Mode==Mec_Car)
			 {
				 //The Mec_car and omni_car show Z-axis angular velocity
				 //���֡�ȫ����С����ʾZ����ٶ�
				 OLED_ShowString(55,0,"GZ");
				 if( imu.gyro.z<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-imu.gyro.z,5,12);
				 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, imu.gyro.z,5,12);		
			 }
             else if(Car_Mode==FourWheel_Car||Car_Mode==Tank_Car||Car_Mode==Akm_Car)
			 {
				 //Akm_Car, Diff_Car, FourWheel_Car and Tank_Car Displays gyroscope zero
				 //�����������١��������Ĵ�����ʾ���������
				 OLED_ShowString(55,0,"BIAS");
				 if( imu.Deviation_gyro.z<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-imu.Deviation_gyro.z,3,12);  //Zero-drift data of gyroscope Z axis
				 else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, imu.Deviation_gyro.z,3,12);	//������z�����Ư������	
			 }
			 //The first line of the display displays the content//
			 //��ʾ����1����ʾ����//
			 

			 //The second line of the display displays the content//
			 //��ʾ����2����ʾ����//
             if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor A
				//���֡�ȫ���֡���������ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,10,"A");
				if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
															OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
				else                 	OLED_ShowString(15,10,"+"),
															OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
				
				if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
															OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,10,"+"),
															OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);

			 }
			 else if(Car_Mode==Tank_Car)
			 {
				 //The Akm_Car, Diff_Car and Tank_Car show Z-axis angular velocity
				 //�����������١�̹��С����ʾZ����ٶ�
				 OLED_ShowString(00,10,"GYRO_Z:");
				 if( imu.gyro.z<0)  OLED_ShowString(60,10,"-"),
												 OLED_ShowNumber(75,10,-imu.gyro.z,5,12);
				 else            OLED_ShowString(60,10,"+"),
												 OLED_ShowNumber(75,10, imu.gyro.z,5,12);			
			 }	 
			 //The second line of the display displays the content//
			 //��ʾ����2����ʾ����//
			 
			 //Lines 3 and 4 of the display screen display content//
			 //��ʾ����3��4����ʾ����//
             if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car)
			 {
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor B
				//���֡�ȫ���֡���������ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,20,"B");		
				if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
				else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
				
				if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
				//Mec_Car, Omni_Car and FourWheel_Car Display the target speed and current actual speed of motor C
				//���֡�ȫ���֡���������ʾ���C��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				OLED_ShowString(0,30,"C");
				if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
				else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
					
				if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
			 }
			 else if(Car_Mode==Tank_Car)
			 {
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor A
				 //�����������١��Ĵ�����ʾ���A��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				 OLED_ShowString(0,20,"L:");
				 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
																OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
				 else                 	OLED_ShowString(15,20,"+"),
																OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12);  
				 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
																OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,20,"+"),
																OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
				 //Akm_Car, Diff_Car and Tank_Car Display the target speed and current actual speed of motor B
				 //�����������١��Ĵ�����ʾ���B��Ŀ���ٶȺ͵�ǰʵ���ٶ�
				 OLED_ShowString(0,30,"R:");
				 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
																OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
				 else                 	OLED_ShowString(15,30,"+"),
																OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12);  
					
				 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
																OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
				 else                 	OLED_ShowString(60,30,"+"),
																OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);

			 }
			 //Lines 3 and 4 of the display screen display content//
			 //��ʾ����3��4����ʾ����//
			 
			 //Line 5 of the display displays the content//
			 //��ʾ����5����ʾ����//
             if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car)
			 {
					//Mec_Car Display the target speed and current actual speed of motor D
					//����С����ʾ���D��Ŀ���ٶȺ͵�ǰʵ���ٶ�
					OLED_ShowString(0,40,"D");
					if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
																OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
					else                 	OLED_ShowString(15,40,"+"),
																OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
					if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
																OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
					else                 	OLED_ShowString(60,40,"+"),
																OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
			 }
			 
			
			 else if(Car_Mode==Tank_Car)
			 {
				 // The Diff_Car and Tank_Car displays the PWM values of the left and right motors
				 //����С�����Ĵ�����ʾ���ҵ����PWM����ֵ
																 OLED_ShowString(00,40,"MA");
				 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
																 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(20,40,"+"),
																 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
																 OLED_ShowString(60,40,"MB");
				 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
																 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
				 else                 	 OLED_ShowString(80,40,"+"),
																 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
			 }
			 //Line 5 of the display displays the content//
			 //��ʾ����5����ʾ����//
				 
			 //Displays the current control mode //��ʾ��ǰ����ģʽ
			 if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");
			 else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			 else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
			 else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
			 else if (Usart1_ON_Flag==1) OLED_ShowString(0,50,"UART1");
			 else if (Usart5_ON_Flag==1) OLED_ShowString(0,50,"UART5");
			 else                       OLED_ShowString(0,50,"ROS  ");
				
			 //Displays whether controls are allowed in the current car
			 //��ʾ��ǰС���Ƿ���������
			 if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  
			 else                      OLED_ShowString(45,50,"OFF"); 
				
																	OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
																	OLED_ShowString(88,50,".");
																	OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
																	OLED_ShowString(110,50,"V");
			 if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
			}
		 
//			/* self-check related */
//			/*�Լ����*/
//			//Display the self-check confirmation screen //��ʾ�Լ�ȷ�Ͻ���
//			if(Check==1&&!Checking&&Checked==0) 
//			{
//					OLED_ShowCheckConfirming();
//			}
//			//Display the interface for self-testing //��ʾ�Լ���н���
//			if(Check==1&&Checking&&Checked==0) 
//			{	
//					OLED_ShowChecking();
//			}	
//			//Show the results of self-test //��ʾ�Լ���
//			if(Check==1&&Checking&&Checked==1) 
//			{		
//				OLED_ShowCheckResult();
//			}
//			/* self-check related */
//			/*�Լ����*/
			
//			//Refresh the screen //ˢ����Ļ
//			if(Check==0)OLED_Refresh_Gram();
//			//The screen refresh rate in self-check mode is reduced by 10 times
//			//�Լ�ģʽ����Ļˢ��Ƶ�ʽ���10��
//			else {if(count>10)OLED_Refresh_Gram(),count=0;}	
	}
	else //΢��ģʽ��ʾ���������ƫ��ֵ 
	{
				//=============��һ��=======================//	
		OLED_ShowString(0,0,"Angle1_init:");
		if( Moveit_Angle1_init<0)	
		{
			OLED_ShowString(100,0,"-");
			OLED_ShowNumber(110,0, -Moveit_Angle1_init,3,12);
		}
		else
		{
			OLED_ShowString(100,00,"+");
			OLED_ShowNumber(110,0, Moveit_Angle1_init,3,12);
		}
						//=============�ڶ���=======================//	
		OLED_ShowString(0,10,"Angle2_init:");
		if( Moveit_Angle2_init<0)	
		{
			OLED_ShowString(100,10,"-");
			OLED_ShowNumber(110,10, -Moveit_Angle2_init,3,12);
		}
		else
		{
			OLED_ShowString(100,10,"+");
			OLED_ShowNumber(110,10, Moveit_Angle2_init,3,12);
		}
				//=============������=======================//	
		OLED_ShowString(0,20,"Angle3_init:");
		if( Moveit_Angle3_init<0)	
		{
			OLED_ShowString(100,20,"-");
			OLED_ShowNumber(110,20, -Moveit_Angle3_init,3,12);
		}
		else
		{
			OLED_ShowString(100,20,"+");
			OLED_ShowNumber(110,20, Moveit_Angle3_init,3,12);
		}
						//=============������=======================//	
		OLED_ShowString(0,30,"Angle4_init:");
		if( Moveit_Angle4_init<0)	
		{
			OLED_ShowString(100,30,"-");
			OLED_ShowNumber(110,30, -Moveit_Angle4_init,3,12);
		}
		else
		{
			OLED_ShowString(100,30,"+");
			OLED_ShowNumber(110,30, Moveit_Angle4_init,3,12);
		}
				//=============������=======================//	
		OLED_ShowString(0,40,"Angle5_init:");
		if( Moveit_Angle5_init<0)	
		{
			OLED_ShowString(100,40,"-");
			OLED_ShowNumber(110,40, -Moveit_Angle5_init,3,12);
		}
		else
		{
			OLED_ShowString(100,40,"+");
			OLED_ShowNumber(110,40, Moveit_Angle5_init,3,12);
		}
				//=============������=======================//	
		OLED_ShowString(0,50,"Angle6_init:");
		if( Moveit_Angle6_init<0)	
		{
			OLED_ShowString(100,50,"-");
			OLED_ShowNumber(110,50, -Moveit_Angle6_init,3,12);
		}
		else
		{
			OLED_ShowString(100,50,"+");
			OLED_ShowNumber(110,50, Moveit_Angle6_init,3,12);
		}


	}
}
	else
	 {
		 if(Proc_Flag==0)							//�û��Լ����
		 {
			 OLED_ShowCHinese(00, 00, "User Self-Test Guide");
			 OLED_ShowCHinese(00, 16, "  User Operation  ");
			 OLED_ShowCHinese(00, 32, "    Confirm Start    ");
			 OLED_ShowString(104, 50, "0/8");
		 }
		 
		 if(Proc_Flag==1)					
		 {
			 OLED_Show_POT();
             OLED_ShowCHinese12(8, 9, "Rotate Zero");
             switch(Car_Mode_Show)
             {
                case Mec_Car:       OLED_ShowString(92,10,"Mec "); break; 
                case FourWheel_Car: OLED_ShowString(92,10,"4WD "); break; 
                case Tank_Car:      OLED_ShowString(92,10,"Tank"); break; 
                case Akm_Car:       OLED_ShowString(92,10,"AKM "); break; 
             }
             OLED_ShowCHinese12(12, 22, "Reset to Zero");
             OLED_ShowCHinese(00, 34, "  Mode Select  ");
			 OLED_ShowString(104, 50, "1/8");
		 }
		 if(Proc_Flag==2)
		 {
             OLED_ShowCHinese(24, 00, "Battery OK");
             OLED_ShowCHinese(00, 16, "Start Gyro Bias");
             OLED_ShowCHinese(24, 32, "Mode Set");
			 OLED_ShowString(104, 50, "2/8");
		 }
		 if(Proc_Flag==3)
		 {
				if( MOTOR_A.Encoder<0)	OLED_ShowString(00,5,"A-"),
																OLED_ShowNumber(20,5,-MOTOR_A.Encoder*1000,5,12);
				else                		OLED_ShowString(00,5,"A+"),
																OLED_ShowNumber(20,5, MOTOR_A.Encoder*1000,5,12);
				
				if( MOTOR_B.Encoder<0)	OLED_ShowString(70,5,"B-"),
																OLED_ShowNumber(90,5,-MOTOR_B.Encoder*1000,5,12);
				else                		OLED_ShowString(70,5,"B+"),
																OLED_ShowNumber(90,5, MOTOR_B.Encoder*1000,5,12);
				
				if( MOTOR_C.Encoder<0)	OLED_ShowString(00,20,"C-"),
																OLED_ShowNumber(20,20,-MOTOR_C.Encoder*1000,5,12);
				else                		OLED_ShowString(00,20,"C+"),
																OLED_ShowNumber(20,20, MOTOR_C.Encoder*1000,5,12);
				
				if( MOTOR_D.Encoder<0)	OLED_ShowString(70,20,"D-"),
																OLED_ShowNumber(90,20,-MOTOR_D.Encoder*1000,5,12);
				else                		OLED_ShowString(70,20,"D+"),
																OLED_ShowNumber(90,20, MOTOR_D.Encoder*1000,5,12);
             OLED_ShowCHinese(24, 32, "Mode Set");
				OLED_ShowString(104, 50, "2/8");
		 }
		 if(Proc_Flag==4)
		 {
             OLED_ShowCHinese(00, 00, "    Warning     ");
             OLED_ShowCHinese(00, 32, "  Mode Running  ");
			 OLED_ShowString(104, 50, "3/8");
		 }
		 if(Proc_Flag==5)
		 {
			 OLED_ShowCHinese(00, 00, "�ζ�С���۲�����");
			 OLED_ShowString(00, 16, "G");
			 if(imu.gyro.x<0)							 OLED_ShowString(16, 16, "-"),
																	 OLED_ShowNumber(24, 16, -imu.gyro.x, 5, 12);
			 else							 					 OLED_ShowString(16, 16, "+"),
																	 OLED_ShowNumber(24, 16, imu.gyro.x, 5, 12);
			 
			 if(imu.gyro.y<0)							 OLED_ShowString(16, 26, "-"),
																	 OLED_ShowNumber(24, 26, -imu.gyro.y, 5, 12);
			 else							 					 OLED_ShowString(16, 26, "+"),
																	 OLED_ShowNumber(24, 26, imu.gyro.y, 5, 12);
			 
			 if(imu.gyro.z<0)	 						OLED_ShowString(16, 36, "-"),
																	OLED_ShowNumber(24, 36, -imu.gyro.z, 5, 12);
			 else						 						OLED_ShowString(16, 36, "+"),
																	OLED_ShowNumber(24, 36, imu.gyro.z, 5, 12);
			 
			 OLED_ShowString(64, 16, "|");
			 OLED_ShowString(64, 26, "|");
			 OLED_ShowString(64, 36, "|");
			 
			 OLED_ShowString(72, 16, "A");
			 if(imu.accel.x<0)							 OLED_ShowString(80, 16, "-"),
																	 OLED_ShowNumber(88, 16, -imu.accel.x, 5, 12);
			 else							 					 OLED_ShowString(80, 16, "+"),
																	 OLED_ShowNumber(88, 16, imu.accel.x, 5, 12);
			 
			 if(imu.accel.y<0)							 OLED_ShowString(80, 26, "-"),
																	 OLED_ShowNumber(88, 26, -imu.accel.y, 5, 12);
			 else							 					 OLED_ShowString(80, 26, "+"),
																	 OLED_ShowNumber(88, 26, imu.accel.y, 5, 12);
			 
			 if(imu.accel.z<0)							 OLED_ShowString(80, 36, "-"),
																	 OLED_ShowNumber(88, 36, -imu.accel.z, 5, 12);
			 else							 					 OLED_ShowString(80, 36, "+"),
																	 OLED_ShowNumber(88, 36, imu.accel.z, 5, 12);
			 OLED_ShowString(104, 50, "3/8");
		 }
		 if(Proc_Flag==6)
		 {
             OLED_ShowCHinese(00, 00, "    Servo Info   ");
             OLED_ShowCHinese(8, 16, "Arm Servo Note");
             OLED_ShowCHinese(16, 32, "Mode: AKM");
             OLED_ShowString(104, 50, "4/8");
		 }
		 if(Proc_Flag==7)
		 {
			 OLED_ShowString(00, 00, "1:"),OLED_ShowNumber(16, 00, Servo_Count[0], 5, 12);
			 OLED_ShowString(64, 00, "2:"),OLED_ShowNumber(80, 00, Servo_Count[1], 5, 12);
			 OLED_ShowString(00, 10, "3:"),OLED_ShowNumber(16, 10, Servo_Count[2], 5, 12);
			 OLED_ShowString(64, 10, "4:"),OLED_ShowNumber(80, 10, Servo_Count[3], 5, 12);
			 OLED_ShowString(00, 20, "5:"),OLED_ShowNumber(16, 20, Servo_Count[4], 5, 12);
			 OLED_ShowString(64, 20, "6:"),OLED_ShowNumber(80, 20, Servo_Count[5], 5, 12);
				OLED_ShowCHinese(16, 32, "ģʽ����е��");
			 OLED_ShowString(104, 50, "4/8");
		 }
		 if(Proc_Flag==8)
		 {
             OLED_ShowCHinese(00, 00, "    Warning     ");
             OLED_ShowCHinese(00, 32, "  Mode Running  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(Proc_Flag==9)
		 {
			 OLED_ShowCHinese(24, 16, "Buzzer");
			 if(Buzzer==1)		OLED_ShowCHinese(88, 16, "ON");
			 else							OLED_ShowCHinese(88, 16, "��");
			 OLED_ShowCHinese(00, 32, "  Mode Running  ");
			 OLED_ShowString(104, 50, "5/8");
		 }
		 if(Proc_Flag==10)
		 {
             OLED_ShowCHinese(00, 00, "  Enable Switch  ");
             OLED_ShowCHinese(32, 16, "State:");
			 if(EN==1)					OLED_ShowCHinese(80, 16, "��");
			 else								OLED_ShowCHinese(80, 16, "��");
			 OLED_ShowCHinese(8, 32, "ģʽ��ʹ�ܿ���");
			 OLED_ShowString(104, 50, "6/8");
		 }
		 if(Proc_Flag==11)
		 {
              OLED_ShowCHinese(00, 00, "Confirm Exit");
              OLED_ShowCHinese(24, 16, "Single Step");
            	OLED_ShowCHinese(24, 32, "Mode Info");
			 OLED_ShowString(104, 50, "7/8");
		 }
		 if(Proc_Flag==12)
		 {
			 OLED_ShowCHinese(8, 00, "Car State");
			 if(uart2_send_flag==1)					OLED_ShowCHinese(48, 16, "Forward");
			 else if(uart2_send_flag==2)		OLED_ShowCHinese(48, 16, "Back");
			 else if(uart2_send_flag==3)		OLED_ShowCHinese(48, 16, "Left");
			 else if(uart2_send_flag==4)		OLED_ShowCHinese(48, 16, "Right");
			 else if(uart2_send_flag==5)		OLED_ShowCHinese(48, 16, "Stop");
//			 OLED_ShowCHinese(00, 00, "��������");
//			 OLED_ShowString16(64, 00, "WHEELTEC");
//			 OLED_ShowCHinese(00, 16, "���յ�");
//			 OLED_ShowString16(48, 16, "ASCII:");
//			 OLED_ShowNumber(96, 18, (USART2->DR), 4, 12);		//��APP���͵����ַ�����������ʹ��Ascii����ʾ
             OLED_ShowCHinese(24, 32, "Mode Info");
			 OLED_ShowString(104, 50, "7/8");
		 }
		 if(Proc_Flag==13)
		 {
				OLED_ShowCHinese(8, 00, "Use USB");
			  OLED_ShowCHinese(20, 16, "Middle");
			  OLED_ShowString16(68, 16, "USB");
				OLED_ShowCHinese(92,16, "Port");
				OLED_ShowCHinese(20, 32, "Mode Info");
			  OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(Proc_Flag==14)
		 {
			 OLED_ShowCHinese(00, 00, "Baud");
			 OLED_ShowString16(36, 00, "#");
			 OLED_ShowCHinese(48, 00, "Tail Mark");
			 
			 OLED_ShowCHinese(8, 16, "Baudrate:");
			 OLED_ShowString16(72, 16, "115200");
//			 OLED_ShowCHinese(00, 00, "��������");
//			 OLED_ShowString16(64, 00, "WHEELTEC");
//			 OLED_ShowCHinese(00, 16, "���յ�");
//			 OLED_ShowString16(48, 16, "ASCII:");
//			 OLED_ShowNumber(96, 18, (USART3->DR), 4, 12);		//�ڴ��������Ϸ��͵����ַ�����������ʹ��Ascii����ʾ
			 OLED_ShowCHinese(20, 32, "ģʽ������");
			 OLED_ShowString16(100,32, "3");
			 OLED_ShowString(104, 50, "8/8");
		 }
		 if(Proc_Flag==15)
		 {
				OLED_ShowCHinese(00, 00, "�����˳��ʼ�ģʽ");
			 OLED_ShowCHinese(00, 16, "Gyro Zero Reset");
			 OLED_ShowCHinese(12, 32, "Current Mode:");
              switch(Car_Mode_Show)
              {
                  case Mec_Car:       OLED_ShowString16(92,32,"Mec "); break; 
                  case FourWheel_Car: OLED_ShowString16(92,32,"4WD "); break; 
                  case Tank_Car:      OLED_ShowString16(92,32,"Tank"); break; 
                  case Akm_Car:       OLED_ShowString16(92,32,"AKM "); break; 
              }
				OLED_ShowString(104, 50, "8/8");
		 }
			 OLED_ShowCHinese12(00, 50, "Double-Click Exit");
	 }
			//=============ˢ��=======================//
		OLED_Refresh_Gram();	
} 
/**************************************************************************
Function: Send data to the APP
Input   : none
Output  : none
�������ܣ���APP��������
��ڲ�������
����  ֵ����
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	
	 //The battery voltage is processed as a percentage
	 //�Ե�ص�ѹ�����ɰٷֱ���ʽ
	 Voltage_Show=(Voltage*1000-10000)/27;
	 if(Voltage_Show>100)Voltage_Show=100; 
	
	 //Wheel speed unit is converted to 0.01m/s for easy display in APP
	 //�����ٶȵ�λת��Ϊ0.01m/s��������APP��ʾ
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	
	 Right_Figure=MOTOR_B.Encoder*100; if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 //Used to alternately print APP data and display waveform
	 //���ڽ����ӡAPP���ݺ���ʾ����
	 flag_show=!flag_show;
	
	 if(PID_Send==1) 
	 {
		 //Send parameters to the APP, the APP is displayed in the debug screen
		 //���Ͳ�����APP��APP�ڵ��Խ�����ʾ
		 		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",
		  (int)((Moveit_Angle1+1.57f)*100), //+1.57����Ϊ���ν��治������ʾ������*100����Ϊ���ν��治��ʵ�������ĵ�������
			(int)((Moveit_Angle2+1.57f)*100),
			(int)((Moveit_Angle3+1.57f)*100),
			(int)((Moveit_Angle4+1.57f)*100),
			(int)((Moveit_Angle5+1.57f)*100),
			(int)((Moveit_Angle6+1.57f)*100),
		  (int)RC_Velocity,
			(int)Velocity_KP,
			(int)Velocity_KI);
		  PID_Send=0;	
	 }	
	 else	if(flag_show==0) 
	 {
		 //Send parameters to the APP and the APP will be displayed on the front page
		 //���Ͳ�����APP��APP����ҳ��ʾ
		 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)imu.gyro.z);
	 }
	 else
	 {
		 //Send parameters to the APP, the APP is displayed in the waveform interface
		 //���Ͳ�����APP��APP�ڲ��ν�����ʾ
		 printf("{B%d:%d:%d}$",(int)imu.gyro.x,(int)imu.gyro.y,(int)imu.gyro.z);
	 }
}


