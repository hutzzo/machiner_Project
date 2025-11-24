#include "balance.h"
#include "arm_servo.h"
#include "pca9685.h"
static void Set_Steering_Servo(int pwm);

u32 Buzzer_count1 = 0;

// Robot mode is wrong to detect flag bits
//������ģʽ�Ƿ��������־λ
int robot_mode_check_flag=0; 

short test_num;

Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����  

u8 command_lost_count=0; //���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //ȫ���ƶ�С���ſ����ٶ�ƽ������
	  if(Car_Mode==Mec_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //�������ٶȽ���ƽ������
  
      //Get the smoothed data 
			//��ȡƽ�������������			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //�����ķ��С��
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//FourWheel car
		//������
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
					
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 	
		}
        else if (Car_Mode==Akm_Car)
        {
            float delta = Steering_Manual_Flag ? Steering_Angle : 0.0f;
            float omega = tanf(delta) * Vx / Axle_spacing;
            float half_track = Wheel_spacing * 0.5f;
            MOTOR_A.Target  = Vx - omega * half_track;
            MOTOR_B.Target  = Vx + omega * half_track;
            MOTOR_C.Target  = 0;
            MOTOR_D.Target  = 0;
            MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude);
            MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude);
        }
		
		//Tank Car
		//�Ĵ���
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		}

		
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//��������100Hz��Ƶ�����У�10ms����һ�Σ�
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			Buzzer_count1++;
			//Time count is no longer needed after 30 seconds
			//ʱ�������30�������Ҫ
			if(SysVal.Time_count<3000)SysVal.Time_count++;
			
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
			Get_Velocity_Form_Encoder();   
			
			if(Servo_init_angle_adjust == 1) Servo_init_angle_adjust_function(); //��������ʼλ��΢��ģʽ
			
			//Click the user button to update the gyroscope zero
				//�����û������������������
				Key();
			
			if(Check==0) //If self-check mode is not enabled //���û�������Լ�ģʽ
			{
//				command_lost_count++; //���ڡ�CAN�������ʧʱ���������ʧ1���ֹͣ����
//				if(command_lost_count>RATE_100_HZ && APP_ON_Flag==0 && Remote_ON_Flag==0 && PS2_ON_Flag==0) //����APP��PS2����ģң��ģʽ������CAN������1������3����ģʽ
//					Move_X=0, Move_Y=0, Move_Z=0;

				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //����APPң������
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //������ģң������
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //����PS2�ֱ���������
				
				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN������1������3(ROS)������5����ֱ�ӵõ�����Ŀ���ٶȣ�������⴦��
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
				 
                
                Drive_Robot_Arm();//��е�ۿ���
                ArmServo_SetTargets(Moveit_PWM1,Moveit_PWM2,Moveit_PWM3,Moveit_PWM4,Moveit_PWM5,Moveit_PWM6);
                ArmServo_Update();
				
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ����������ʧ�ܱ�־λΪ0
				if(Turn_Off(Voltage)==0) 
				 { 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //�ٶȱջ����Ƽ�������PWMֵ��PWM��������ʵ��ת��
					 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
					 
					 Limit_Pwm(16700);//������Ƶ�PWM�޷�������16800
					 
						 
					 
                     //Set different PWM control polarity according to different car models
					 //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm,Position1,Position2,Position3,Position4,Position5,Position6); break; //Mecanum wheel car       //�����ķ��С��
							case FourWheel_Car: Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm,Position1,Position2,Position3,Position4,Position5,Position6); break; //FourWheel car           //������ 
                            case Akm_Car:       Set_Steering_Servo(SERVO_PWM_VALUE(Steering_Manual_Flag?Steering_Angle:Vz_to_Akm_Angle(Move_X, Move_Z))); Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  0, 0, Position1,Position2,Position3,Position4,Position5,Position6); break;
							case Tank_Car:      Set_Pwm(MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,   MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm,Position1,Position2,Position3,Position4,Position5,Position6); break; //Tank Car                //�Ĵ���
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //���Turn_Off(Voltage)����ֵΪ1������������С�������˶���PWMֵ����Ϊ0
				 else	Set_Pwm(0,0,0,0,0,0,0,0,0,0); 
			 }
			else
				{
					if(Proc_Flag==3)						//�Լ���
				{
					 if(check_time_count_motor_forward>0)
					 {	 
						 check_time_count_motor_forward--;
						 Full_rotation=16799;
					 }
					 else if(check_time_count_motor_retreat>0)
					 {	 
							check_time_count_motor_retreat--;
							Full_rotation=-16799;
					 }		
					 
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0, 0, 0, 0, 0, 0); break; //Mecanum wheel car       //�����ķ��С��
							case FourWheel_Car: Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0, 0, 0, 0, 0, 0); break; //FourWheel car           //������ 
                            case Akm_Car:       Set_Pwm( Full_rotation, -Full_rotation, 0, 0, 0, 0, 0, 0, 0, 0); break;
							case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0, 0, 0, 0, 0, 0); break; //Tank Car                //�Ĵ���
					 } 
					 if(!(check_time_count_motor_retreat>0) && !(check_time_count_motor_forward>0))
					 {	 
						 Set_Pwm(0,0,0,0,0, 0, 0, 0, 0, 0);		 
					 }
				}
				if(Proc_Flag==4)		Set_Pwm(0,0,0,0,0, 0, 0, 0, 0, 0);
				if(Proc_Flag==6)		
				{
					if(TIM8_Servo_flag==0)					TIM8_SERVO_Init(9999,168-1);					//��·���
					TIM8_Servo_flag++;
//					Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 1500;
//					Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
				}
				if(Proc_Flag==7)																					//���ƶ��
				{
					/******************************************
					 * ����ӿڣ���е�۴������ϼ�����Ϊ1-6��
					 * 		B15			TIM12->CH2				6
					 * 		B14			TIM12->CH1				5
					 * 		C9			TIM8->CH4					4
					 * 		C8			TIM8->CH3					3
					 * 		C7			TIM8->CH2					2
					 * 		C6			TIM8->CH1					1
					 ******************************************/
					Arm_Action();																//��е�۰ڶ����Ժ���
					Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
				}
				if(Proc_Flag==8)																	//
				{
					 Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 1500;
					 Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
					 Arm_direction = 0;
//					 Num = 100;
					 servo_flag = 0;
				}
				if(Proc_Flag==9)																	//���������1s��һ��
				{
					if((Buzzer_count1/100)%2)			Buzzer = 1;
					else													Buzzer = 0;
				}
				if(Proc_Flag==10)			Buzzer = 0;
//				if(Proc_Flag==13)																	//��APP����WHEELTEC
//				{
//					printf("{#WHEELTEC}$");
//					Proc_Flag++;
//				}
				if(Proc_Flag==14)
				{
					if(uart3_send_flag==1)
					{
						USART3_Return();
						uart3_send_flag = 0;
						message_count = 0;
					}
				}
				}			
		 }  
}

/**************************************************************************
Function: Assign a value to the PWM register to control the rotation of the steering gear on the mechanical arm
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������ƻ�е���϶��ת��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Mechanical_Arm(int Servo1,int Servo2,int Servo3,int Servo4,int Servo5,int Servo6)
{
    uint16_t c1=(uint16_t)((Servo1*4096)/20000);
    uint16_t c2=(uint16_t)((Servo2*4096)/20000);
    uint16_t c3=(uint16_t)((Servo3*4096)/20000);
    uint16_t c4=(uint16_t)((Servo4*4096)/20000);
    uint16_t c5=(uint16_t)((Servo5*4096)/20000);
    uint16_t c6=(uint16_t)((Servo6*4096)/20000);
    PCA9685_SetPWM(0,0,c1);
    PCA9685_SetPWM(1,0,c2);
    PCA9685_SetPWM(2,0,c3);
    PCA9685_SetPWM(3,0,c4);
    PCA9685_SetPWM(4,0,c5);
    PCA9685_SetPWM(5,0,c6);
}

static void Set_Steering_Servo(int pwm)
{
    TIM8->CCR1 = pwm;
}

/**************************************************************************
Function: The action of the robotic arm in self-check mode
Input   : none
Output  : none
�������ܣ��Լ�ģʽ�»�е�۵Ķ���
��ڲ�������
����  ֵ����
**************************************************************************/
void Arm_Action(void)
{
	if(servo_flag==0)							
	{
		if(Arm_direction<=2)						
		{
			if(Servo_Count[0]>1500)					Servo_Count[0] -= 10;				//1
			else if(Servo_Count[0]<1500)		Servo_Count[0] += 10;
			
			if(Servo_Count[1]>1500)					Servo_Count[1] -= 10;				//2
			else if(Servo_Count[1]<1500)		Servo_Count[1] += 10;
			
			if(Servo_Count[2]>1500)					Servo_Count[2] -= 10;				//3
			else if(Servo_Count[2]<1500)		Servo_Count[2] += 10;
			
			if(Servo_Count[3]>1500)					Servo_Count[3] -= 10;				//4
			else if(Servo_Count[3]<1500)		Servo_Count[3] += 10;
			
			if(Servo_Count[4]>1500)					Servo_Count[4] -= 10;				//5
			else if(Servo_Count[4]<1500)		Servo_Count[4] += 10;
			
			if(Servo_Count[5]>1000)					Servo_Count[5] -= 10;				//6
			else if(Servo_Count[5]<1000)		Servo_Count[5] += 10;
		}
		if(Servo_Count[0]==1500&&Servo_Count[1]==1500&&Servo_Count[2]==1500&&Servo_Count[3]==1500&&Servo_Count[4]==1500&&Servo_Count[5]==1000)			servo_flag++;
	}
	else if(servo_flag==1)
	{
		if(Arm_direction==0)					
		{
			if(Servo_Count[0]<2500)			Servo_Count[0] += 10;
			else												servo_flag++;
		}
		else if(Arm_direction==1)			
		{
			if(Servo_Count[0]>500)			Servo_Count[0] -= 10;
			else												servo_flag++;
		}
	}
	else if(servo_flag==2)
	{
		if(Arm_direction<2)						
		{
			if(Servo_Count[1]>1300)					Servo_Count[1] -= 10;
			else														servo_flag++;
		}
	}
	else if(servo_flag==3)
	{
		if(Arm_direction<2)	
		{
			if(Servo_Count[2]>650)					Servo_Count[2] -= 10;
			else														servo_flag++;
		}
	}
	else if(servo_flag==4)
	{
		if(Arm_direction<2)						
		{
			if(Servo_Count[3]>650)					Servo_Count[3] -= 10;
			else														servo_flag++;
		}
	}
	else if(servo_flag==5)
	{
		if(Arm_direction<2)				
		{
			if(Servo_Count[4]>500)					Servo_Count[4] -= 10;
			else														servo_flag++;
		}
	}
	else if(servo_flag==6)
	{
		if(Arm_direction<2)	
		{
			if(Servo_Count[5]<1500)					Servo_Count[5] += 10;
			else														servo_flag = 0,Arm_direction++;
		}		
	}
}

/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo1,int servo2,int servo3,int servo4,int servo5,int servo6)
{
	//Forward and reverse control of motor A (reversed)
	if(motor_a<0)			PWMA2=16799,PWMA1=16799+motor_a;
	else 	            PWMA1=16799,PWMA2=16799-motor_a;
	
	//Forward and reverse control of motor
	//�������ת����	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//�������ת����	
	if(motor_c<0)			PWMC1=16799,PWMC2=16799+motor_c;
	else 	            PWMC2=16799,PWMC1=16799-motor_c;
	
	//Forward and reverse control of motor
	//�������ת����
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//�������
	
//	Velocity1=Position_PID1(Position1,servo1);//���PID����
//	Velocity2=Position_PID2(Position2,servo2);//���PID����
//	Velocity3=Position_PID3(Position3,servo3);//���PID����
//	Velocity4=Position_PID4(Position4,servo4);//���PID����
//	Velocity5=Position_PID5(Position5,servo5);//���PID����
//	Velocity6=Position_PID6(Position6,servo6);//���PID����

//	Position1+=Velocity1;		   //�ٶȵĻ��֣��õ������λ��
//	Position2+=Velocity2;
//	Position3+=Velocity3;
//	Position4+=Velocity4;
//	Position5+=Velocity5;	
	//	Position6+=Velocity6;		

		if(Car_Mode!=Akm_Car)
		{
			Servo_PWM1 =servo1;
			Servo_PWM2 =servo2;
			Servo_PWM3 =servo3;
			Servo_PWM4 =servo4;
			Servo_PWM5 =servo5;
			Servo_PWM6 =servo6;
		}

		if(Moveit_Active_Counter>0)
		{
			Set_Mechanical_Arm(servo1,servo2,servo3,servo4,servo5,servo6);
			Moveit_Active_Counter--;
		}
		else
		{
			Set_Mechanical_Arm(SERVO_INIT,SERVO_INIT,SERVO_INIT,SERVO_INIT,SERVO_INIT,SERVO_INIT);
		}
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬������ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ��������ƣ�1����������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16700)Pwm=16700;
	 if(Pwm<-16700)Pwm=-16700;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}

/**************************************************************************
�������ܣ�λ��ʽPID������
��ڲ���������������λ����Ϣ��Ŀ��λ��
����  ֵ�����PWM
����λ��ʽ��ɢPID��ʽ 
pwm=Kp*e(k)+Ki*��e(k)+Kd[e��k��-e(k-1)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  
��e(k)����e(k)�Լ�֮ǰ��ƫ����ۻ���;����kΪ1,2,,k;
pwm�������
**************************************************************************/
float Position_PID1 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID2 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID3 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID4 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID5 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
float Position_PID6 (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Encoder;                                  //����ƫ��
	 Integral_bias+=Bias;	                                 //���ƫ��Ļ���
	 Pwm=Position_KP*Bias/100+Position_KI*Integral_bias/100+Position_KD*(Bias-Last_Bias)/100;       //λ��ʽPID������
	 Last_Bias=Bias;                                       //������һ��ƫ�� 
	 return Pwm;                                           //�������
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car) //The omnidirectional wheel moving trolley can move laterally //ȫ�����˶�С�����Խ��к����ƶ�
	{
	 switch(Flag_Direction)  //Handle direction control commands //���������������
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //����޷������ָ����ת�����״̬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //����ת  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //����ת
		 else 		               Move_Z=0;                       //stop           //ֹͣ
	 }
	}	
	else //Non-omnidirectional moving trolley //��ȫ���ƶ�С��
	{
	 switch(Flag_Direction) //Handle direction control commands //���������������
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	
	}
	
 if(Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
		float step1=0.005;	
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
	
	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
//	  if (PS2_KEY==11)		RC_Velocity+=5;  //To accelerate//����
//	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //����	
	
//		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //��PS2�ֱ�����������д���
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
	  Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z������ת��
	  if(Car_Mode==Mec_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	

        else if(Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
		
				if(PS2_LX<100 && PS2_RX>200)  Move_X=0,Move_Y=0,Move_Z=0;  //������˳����΢��ģʽʱ,��ֹС������
		else if(PS2_LX>200 && PS2_RX<100)   Move_X=0,Move_Y=0,Move_Z=0;
		
		
		if(Servo_init_angle_adjust == 0)//��������
		{
			// PS2 control now modifies target angles for smooth transition
			// PS2���Ƶ��ڽ��޸�Ŀ��Ƕȣ�ʵ��ƽ��ת��
			switch(PS2_KEY)   //�������
			 { 
				case 9:       Moveit_Target_Angle1=Moveit_Target_Angle1+step1;  break; 
        case 11:      Moveit_Target_Angle1=Moveit_Target_Angle1-step1;  break;				 
				case 7:       Moveit_Target_Angle2=Moveit_Target_Angle2+step1;  break; 
        case 5:       Moveit_Target_Angle2=Moveit_Target_Angle2-step1;  break;				 
				case 8:       Moveit_Target_Angle3=Moveit_Target_Angle3+step1;  break;
				case 6:       Moveit_Target_Angle3=Moveit_Target_Angle3-step1;  break;  
				case 15:      Moveit_Target_Angle4=Moveit_Target_Angle4+step1;  break;       
				case 13:      Moveit_Target_Angle4=Moveit_Target_Angle4-step1;  break;   
				case 16:      Moveit_Target_Angle5=Moveit_Target_Angle5-step1;  break;   
				case 14:      Moveit_Target_Angle5=Moveit_Target_Angle5+step1;  break; 
				case 12:      Moveit_Target_Angle6=Moveit_Target_Angle6-step1;  break;   
				case 10:      Moveit_Target_Angle6=Moveit_Target_Angle6+step1;  break;
				default:        break;
			 }

	  }
		
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 

/**************************************************************************
Function: The remote control command of model aircraft is processed
Input   : none
Output  : none
�������ܣ��Ժ�ģң�ؿ���������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Remote_Control(void)
{
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    static u8 thrice=100; 
    int Threshold=100; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���

	  //limiter //�޷�
    int LX,LY,RY,RX,Remote_RCvelocity; 
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

	  // Front and back direction of left rocker. Control forward and backward.
	  //��ҡ��ǰ���򡣿���ǰ�����ˡ�
    LX=Remoter_Ch2-1500; 
	
	  //Left joystick left and right.Control left and right movement. Only the wheelie omnidirectional wheelie will use the channel.
	  //Ackerman trolleys use this channel as a PWM output to control the steering gear
	  //��ҡ�����ҷ��򡣿��������ƶ�������ȫ���ֲŻ�ʹ�õ���ͨ����������С��ʹ�ø�ͨ����ΪPWM������ƶ��
    LY=Remoter_Ch4-1500;

    //Front and back direction of right rocker. Throttle/acceleration/deceleration.
		//��ҡ��ǰ��������/�Ӽ��١�
	  RX=Remoter_Ch3-1500;

    //Right stick left and right. To control the rotation. 
		//��ҡ�����ҷ��򡣿�����ת��
    RY=Remoter_Ch1-1500; 

    if(LX>-Threshold&&LX<Threshold)LX=0;
    if(LY>-Threshold&&LY<Threshold)LY=0;
    if(RX>-Threshold&&RX<Threshold)RX=0;
	  if(RY>-Threshold&&RY<Threshold)RY=0;
		
		//Throttle related //�������
		Remote_RCvelocity=RC_Velocity+RX;
	  if(Remote_RCvelocity<0)Remote_RCvelocity=0;
		
		//The remote control command of model aircraft is processed
		//�Ժ�ģң�ؿ���������д���
    Move_X= LX*Remote_RCvelocity/500; 
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/2)/500;      
			 
		//Z������ת��
	  if(Car_Mode==Mec_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}	
        else if(Car_Mode==Tank_Car||Car_Mode==FourWheel_Car||Car_Mode==Akm_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*Remote_RCvelocity/500;
		}
		
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;       
    Move_Y=Move_Y/1000;      
		Move_Z=Move_Z;
		
	  //Data within 1 second after entering the model control mode will not be processed
	  //�Խ��뺽ģ����ģʽ��1���ڵ����ݲ�����
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;
				
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
Function: Click the user button to update gyroscope zero
Input   : none
Output  : none
�������ܣ������û������������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(Check==0)
	{
		if(tmp==2)ImuData_copy(&imu.Deviation_gyro,&imu.gyro),ImuData_copy(&imu.Deviation_accel,&imu.accel);
	}
	else if(Check==1)
	{
		if(tmp==1)
		{
			Proc_Flag++;
			if(Proc_Flag==16)			
			{
				Check = 0;
				Buzzer = 0;
				Proc_Flag = 0;
				check_time_count_motor_forward=300;
				check_time_count_motor_retreat=500;
				Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 1500;
				Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
				servo_flag = 0;
				Arm_direction = 0;
				TIM8_Servo_flag = 0;
				TIM8_Cap_Init(9999,168-1);  //�߼���ʱ��TIM8��ʱ��Ƶ��Ϊ168M
			}
		}
		else if(tmp==2)
		{
			Check = 0;
			Buzzer = 0;
			Proc_Flag = 0;
			check_time_count_motor_forward=300;
			check_time_count_motor_retreat=500;
			Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 1500;
			Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
			servo_flag = 0;
			Arm_direction = 0;
			TIM8_Servo_flag = 0;
			TIM8_Cap_Init(9999,168-1);  //�߼���ʱ��TIM8��ʱ��Ƶ��Ϊ168M  
		}
	}
}
/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	//test_num=OriginalEncoder.B;
	
	  //Decide the encoder numerical polarity according to different car models
		//���ݲ�ͬС���ͺž�����������ֵ����
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A;  Encoder_B_pr= OriginalEncoder.B;  Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A;  Encoder_B_pr= OriginalEncoder.B;  Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Akm_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B;  Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr=-OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A;  Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr=  OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D;  break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}
/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת����ֹͣʹ��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//�������6�νӽ�����������ж�Ϊ�����ת���õ��ʧ��	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

/**************************************************************************
�������ܣ���е�۹ؽڽǶ�ת��Ӧpwmֵ����
��ڲ�������е��Ŀ��ؽڽǶ�
����  ֵ���ؽڽǶȶ�Ӧ��pwmֵ
**************************************************************************/
int SERVO_PWM_VALUE(float angle)
{
			int K=1000;
			float Ratio=0.64;
			int pwm_value;
			
			pwm_value=(SERVO_INIT-angle*K*Ratio); //���Ŀ��
	    return pwm_value;
}
/**************************************************************************
�������ܣ���е�۹ؽڽǶ���λ����
��ڲ�������
����  ֵ����
**************************************************************************/
void moveit_angle_limit(void)
{ 
	Moveit_Angle1=target_limit_float(Moveit_Angle1,-1.57,1.57);
	Moveit_Angle2=target_limit_float(Moveit_Angle2,-1.57,1.57);
	Moveit_Angle3=target_limit_float(Moveit_Angle3,-1.57,1.57);
	Moveit_Angle4=target_limit_float(Moveit_Angle4,-0.3,1.57);
	Moveit_Angle5=target_limit_float(Moveit_Angle5,-1.57,1.57);
	Moveit_Angle6=target_limit_float(Moveit_Angle6,-0.7,0.7);  //��צ���˶���Χ��С
}
/**************************************************************************
�������ܣ���е�۹ؽ�PWMֵ��λ����
��ڲ�������
����  ֵ����
**************************************************************************/
void moveit_pwm_limit(void)
{ 
	Moveit_PWM1=target_limit_int(Moveit_PWM1,400,2600);
	Moveit_PWM2=target_limit_int(Moveit_PWM2,400,2600);
	Moveit_PWM3=target_limit_int(Moveit_PWM3,400,2600);
	Moveit_PWM4=target_limit_int(Moveit_PWM4,400,2600);
	Moveit_PWM5=target_limit_int(Moveit_PWM5,400,2600);
	Moveit_PWM6=target_limit_int(Moveit_PWM6,900,2100);  
}
/**************************************************************************
�������ܣ�ƽ������Ƕȹ��ɣ����ֹ�ؽڶ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void smooth_angle_transition(void)
{
	float delta;
	
	// Joint 1 smooth transition �ؽ�1ƽ��ת��
	delta = Moveit_Target_Angle1 - Moveit_Angle1;
	if(delta > Moveit_Max_Speed) 
		Moveit_Angle1 += Moveit_Max_Speed;
	else if(delta < -Moveit_Max_Speed) 
		Moveit_Angle1 -= Moveit_Max_Speed;
	else 
		Moveit_Angle1 = Moveit_Target_Angle1;
	
	// Joint 2 smooth transition �ؽ�2ƽ��ת��
	delta = Moveit_Target_Angle2 - Moveit_Angle2;
	if(delta > Moveit_Max_Speed) 
		Moveit_Angle2 += Moveit_Max_Speed;
	else if(delta < -Moveit_Max_Speed) 
		Moveit_Angle2 -= Moveit_Max_Speed;
	else 
		Moveit_Angle2 = Moveit_Target_Angle2;
	
	// Joint 3 smooth transition �ؽ�3ƽ��ת��
	delta = Moveit_Target_Angle3 - Moveit_Angle3;
	if(delta > Moveit_Max_Speed) 
		Moveit_Angle3 += Moveit_Max_Speed;
	else if(delta < -Moveit_Max_Speed) 
		Moveit_Angle3 -= Moveit_Max_Speed;
	else 
		Moveit_Angle3 = Moveit_Target_Angle3;
	
	// Joint 4 smooth transition �ؽ�4ƽ��ת��
	delta = Moveit_Target_Angle4 - Moveit_Angle4;
	if(delta > Moveit_Max_Speed) 
		Moveit_Angle4 += Moveit_Max_Speed;
	else if(delta < -Moveit_Max_Speed) 
		Moveit_Angle4 -= Moveit_Max_Speed;
	else 
		Moveit_Angle4 = Moveit_Target_Angle4;
	
	// Joint 5 smooth transition �ؽ�5ƽ��ת��
	delta = Moveit_Target_Angle5 - Moveit_Angle5;
	if(delta > Moveit_Max_Speed) 
		Moveit_Angle5 += Moveit_Max_Speed;
	else if(delta < -Moveit_Max_Speed) 
		Moveit_Angle5 -= Moveit_Max_Speed;
	else 
		Moveit_Angle5 = Moveit_Target_Angle5;
	
	// Joint 6 smooth transition �ؽ�6ƽ��ת��
	delta = Moveit_Target_Angle6 - Moveit_Angle6;
	if(delta > Moveit_Max_Speed) 
		Moveit_Angle6 += Moveit_Max_Speed;
	else if(delta < -Moveit_Max_Speed) 
		Moveit_Angle6 -= Moveit_Max_Speed;
	else 
		Moveit_Angle6 = Moveit_Target_Angle6;
}

/**************************************************************************
�������ܣ���е�۹ؽ��������ִ��룬ʹ��PIDλ�ÿ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void Drive_Robot_Arm(void)
{
		  smooth_angle_transition(); //ƽ������Ƕȹ���
		  moveit_angle_limit();		 //�ؽڽǶ����޷�
			Moveit_PWM1=  SERVO_PWM_VALUE(Moveit_Angle1)+Moveit_Angle1_init; //����Ŀ�껡�ȣ�������Ŀ��PWMֵ
			Moveit_PWM2 = SERVO_PWM_VALUE(Moveit_Angle2)+Moveit_Angle2_init;
			Moveit_PWM3 = SERVO_PWM_VALUE(Moveit_Angle3)+Moveit_Angle3_init;
			Moveit_PWM4 = SERVO_PWM_VALUE(Moveit_Angle4)+Moveit_Angle4_init;
			Moveit_PWM5 = SERVO_PWM_VALUE(Moveit_Angle5)+Moveit_Angle5_init;
			Moveit_PWM6 = SERVO_PWM_VALUE(Moveit_Angle6)+Moveit_Angle6_init;
	    moveit_pwm_limit();
	
			Velocity1=Position_PID1(Position1,Moveit_PWM1);
	    Velocity2=Position_PID2(Position2,Moveit_PWM2);
	    Velocity3=Position_PID3(Position3,Moveit_PWM3);
	    Velocity4=Position_PID4(Position4,Moveit_PWM4);
	    Velocity5=Position_PID5(Position5,Moveit_PWM5);
      Velocity6=Position_PID6(Position6,Moveit_PWM6);

      Position1+=Velocity1;		   //�ٶȵĻ��֣��õ������λ��
      Position2+=Velocity2;
      Position3+=Velocity3;
	    Position4+=Velocity4;
	    Position5+=Velocity5;	
	    Position6+=Velocity6;		
}
