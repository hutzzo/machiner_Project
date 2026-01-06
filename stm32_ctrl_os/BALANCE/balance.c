#include "balance.h"
#include "arm_servo.h"
#include "pca9685.h"
#include "pstwo.h"
#include "motor.h"

u32 Buzzer_count1 = 0;

// Robot mode is wrong to detect flag bits
//模式是否错误标志位
int robot_mode_check_flag=0; 

short test_num;

Encoder OriginalEncoder; //Encoder raw data //编码器原始数据  

int command_lost_count=0; //控制指令丢失计数器
float g_amplitude = 10.0f; // Limit for target speed (m/s)

static void Set_Steering_Servo(int pwm)
{
    Servo_PWM1 = pwm; // Channel 1 of TIM8
}

/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
功能描述：运动学逆解，根据三轴目标速度计算各轮目标转速
输入参数：X和Y，Z轴方向的目标运动速度
返 回 值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		//Set the target speed according to different models
		//根据不同车型设置目标速度
		if(Car_Mode==Mec_Car || Car_Mode==FourWheel_Car) 
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  =  Vx + Vy - Vz * (Axle_spacing + Wheel_spacing);
			MOTOR_B.Target = -Vx + Vy - Vz * (Axle_spacing + Wheel_spacing);
			MOTOR_C.Target = -Vx + Vy + Vz * (Axle_spacing + Wheel_spacing);
			MOTOR_D.Target =  Vx + Vy + Vz * (Axle_spacing + Wheel_spacing);
			
			//Wheel (motor) target speed limit //轮子(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-g_amplitude,g_amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-g_amplitude,g_amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-g_amplitude,g_amplitude); 
	    MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-g_amplitude,g_amplitude); 
		}
		else if(Car_Mode==Tank_Car)
		{
			//Inverse kinematics //运动学逆解
			MOTOR_A.Target  = -Vx + Vz * (Wheel_spacing); //左
			MOTOR_B.Target =  Vx + Vz * (Wheel_spacing); //右
			MOTOR_C.Target =  0; //Tank car only has 2 wheels //坦克车只有2个轮子
			MOTOR_D.Target =  0; 
			
			//Wheel (motor) target speed limit //轮子(电机)目标速度限幅
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-g_amplitude,g_amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-g_amplitude,g_amplitude); 
			MOTOR_C.Target=0; //Out of use //没有使用的
			MOTOR_D.Target=0; //Out of use //没有使用的
		}
	else if(Car_Mode==Akm_Car)
	{
		//Inverse kinematics //运动学逆解
		// 注意：实际硬件中 B在左边，A在右边
		if(WheelSpeed_Manual_Flag)
		{
			MOTOR_B.Target = Left_Target_mps;   // B在左边
			MOTOR_A.Target = Right_Target_mps;  // A在右边
		}
		else
		{
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    // B=左轮目标速度
			MOTOR_A.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    // A=右轮目标速度
		}
		
		//Wheel (motor) target speed limit //轮子(电机)目标速度限幅
	  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-g_amplitude,g_amplitude); 
	  MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-g_amplitude,g_amplitude); 
		MOTOR_C.Target=0; //Out of use //没有使用的
		MOTOR_D.Target=0; //Out of use //没有使用的
	}
}

/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
功能描述：FreeRTOS任务，核心运动控制任务
输入参数：无
返 回 值：无
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();

    while(1)
    {	
			// This task runs at a frequency of 100Hz (10ms control once)
			//该任务以100Hz的频率运行（10ms控制一次）
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
			Buzzer_count1++;
			//Time count is no longer needed after 30 seconds
			//时间计数，30秒后不再需要
			if(SysVal.Time_count<3000)SysVal.Time_count++;
			
			//Get the encoder data, that is, the real time wheel speed, 
			//and convert to transposition international units
			//获取编码器数据，即实时轮速，并转换为国际单位
			Get_Velocity_Form_Encoder();   
			
			if(Servo_init_angle_adjust == 1) Servo_init_angle_adjust_function(); //进入舵机初始位置微调模式
			
			//Click the user button to update the gyroscope zero
				//点击用户按键更新陀螺仪零点
				Key();
			
			// Read battery voltage
			Voltage = Get_battery_volt();
			
			if(Check==0) //If self-check mode is not enabled //如果没有进入自检模式
			{
				// 【调试代码已注释】硬件测试完成，电机能正常转动
				// 现在使用正常的开环控制逻辑（由串口指令控制）
				// TIM10->CCR1 = 8000;  
				// TIM11->CCR1 = 0;     
				// TIM9->CCR1  = 8000;  
				// TIM9->CCR2  = 0;     
				// PWMA1 = 8000;  
				// PWMB1 = 8000;
				// PWMA2 = 0;
				// PWMB2 = 0;
				
				// 防止计数器溢出
				if(command_lost_count < 500) command_lost_count++; 
				
				// 超时保护：如果超过1秒没有收到任何控制指令，清除所有控制标志并强制停车
				if(command_lost_count>RATE_100_HZ) 
				{
					// 清除所有控制标志
					APP_ON_Flag=0;
					Remote_ON_Flag=0;
					PS2_ON_Flag=0;
					Usart1_ON_Flag=0;
					Usart3_ON_Flag=0;  // 【修复】串口3超时也要清除
					Usart5_ON_Flag=0;
					CAN_ON_Flag=0;
					// 清零所有速度指令
					Move_X=0; Move_Y=0; Move_Z=0;
					Left_Target_mps=0;
					Right_Target_mps=0;
					WheelSpeed_Manual_Flag=0;
				}

				if      (APP_ON_Flag)      Get_RC();         //Handle the APP remote commands //处理APP遥控指令
				else if (Remote_ON_Flag)   Remote_Control(); //Handle model aircraft remote commands //处理航模遥控指令
				else if (PS2_ON_Flag)      PS2_control();    //Handle PS2 controller commands //处理PS2手柄指令
				
				//CAN, Usart 1, Usart 3, Uart5 control can directly get the three axis target speed, 
				//without additional processing
				//CAN、串口1、串口3(ROS)、串口5可直接得到三轴目标速度，无需额外处理
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);
				
                Drive_Robot_Arm();//机械臂控制函数
                ArmServo_SetTargets(Moveit_PWM1,Moveit_PWM2,Moveit_PWM3,Moveit_PWM4,Moveit_PWM5,Moveit_PWM6);
                ArmServo_Update();
				
				// 【修复】转向舵机控制独立于电压检查，确保舵机始终能响应
				// Steering servo control is independent of voltage check
				if(Car_Mode==Akm_Car)
				{
					Set_Steering_Servo(SERVO_PWM_VALUE(Steering_Manual_Flag?Steering_Angle:Vz_to_Akm_Angle(Move_X, Move_Z)));
				}
				
				//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
        //and the software failure flag is 0
				//如果电池电压没有异常，且使能开关在ON位置，且软件失败标志位为0
				if(Turn_Off(Voltage)==0)  // 恢复电压检查，低于11V停止输出 
				 { 			
           //Speed closed-loop control to calculate the PWM value of each motor, 
					 //PWM represents the actual wheel speed					 
					 //速度闭环控制计算各电机PWM值，PWM代表实际轮速
					 
					 // 【调试模式】直接开环控制，绕过PI控制器和编码器反馈
					 // 正式使用时注释掉这4行，恢复下面被注释的PI控制
					 // 注意：Target单位是m/s，系数要够大才能产生足够的PWM（推荐20000-80000）
					 MOTOR_A.Motor_Pwm = (int)(MOTOR_A.Target * 50000.0f);  // 改为50000，因为Target是m/s
					 MOTOR_B.Motor_Pwm = (int)(MOTOR_B.Target * 50000.0f);
					 MOTOR_C.Motor_Pwm = (int)(MOTOR_C.Target * 50000.0f);
					 MOTOR_D.Motor_Pwm = (int)(MOTOR_D.Target * 50000.0f);
					 
					 // 【正常模式】闭环PI控制（调试时被注释）
					 //MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
					 //MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
					 //MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
					 //MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
					 
					 Limit_Pwm(16700);//限制频率PWM限幅，最大16800
					 
                     //Set different PWM control polarity according to different car models
					 //根据不同车型设置不同PWM控制极性
					 switch(Car_Mode)
					 {
							case Mec_Car:       Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm,Position1,Position2,Position3,Position4,Position5,Position6); break; //Mecanum wheel car       //麦克纳姆轮小车
							case FourWheel_Car: Set_Pwm(MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  -MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm,Position1,Position2,Position3,Position4,Position5,Position6); break; //FourWheel car           //四轮车 
                            case Akm_Car:       Set_Pwm(MOTOR_A.Motor_Pwm, MOTOR_B.Motor_Pwm,  0, 0, Position1,Position2,Position3,Position4,Position5,Position6); break; // B电机反向安装，改为正号
							case Tank_Car:      Set_Pwm(MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,   MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm,Position1,Position2,Position3,Position4,Position5,Position6); break; //Tank Car                //坦克车
					 }
				 }
				 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
				 //如果Turn_Off(Voltage)返回值为1，则不允许小车运动，PWM值设为0
				 else	
				 {
					 Set_Pwm(0,0,0,0,0,0,0,0,0,0);
					 // 电压异常时，舵机回中（可选，也可以保持当前位置）
					 // if(Car_Mode==Akm_Car) Set_Steering_Servo(SERVO_INIT);
				 } 
			 }
			else
				{
					if(Proc_Flag==3)						//自检模式
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
							case Mec_Car:       Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0, 0, 0, 0, 0, 0); break; //Mecanum wheel car       //麦克纳姆轮小车
							case FourWheel_Car: Set_Pwm( Full_rotation, -Full_rotation, -Full_rotation, Full_rotation, 0, 0, 0, 0, 0, 0); break; //FourWheel car           //四轮车 
                            case Akm_Car:       Set_Pwm( Full_rotation, -Full_rotation, 0, 0, 0, 0, 0, 0, 0, 0); break;
							case Tank_Car:      Set_Pwm( Full_rotation,  Full_rotation,  Full_rotation, Full_rotation, 0, 0, 0, 0, 0, 0); break; //Tank Car                //坦克车
					 } 
					 if(!(check_time_count_motor_retreat>0) && !(check_time_count_motor_forward>0))
					 {	 
						 Set_Pwm(0,0,0,0,0, 0, 0, 0, 0, 0);		 
					 }
				}
				if(Proc_Flag==4)		Set_Pwm(0,0,0,0,0, 0, 0, 0, 0, 0);
				if(Proc_Flag==6)		
				{
					if(TIM8_Servo_flag==0)					TIM8_SERVO_Init(9999,168-1);					//初始化舵机
					TIM8_Servo_flag++;
				}
				if(Proc_Flag==7)																					//机械臂动作
				{
					Arm_Action();																//机械臂动作执行函数
					Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
				}
				if(Proc_Flag==8)																	//复位
				{
					 Servo_Count[0] = Servo_Count[1] = Servo_Count[2] = Servo_Count[3] = Servo_Count[4] = Servo_Count[5] = 1500;
					 Set_Mechanical_Arm(Servo_Count[0],Servo_Count[1],Servo_Count[2],Servo_Count[3],Servo_Count[4],Servo_Count[5]);
					 Arm_direction = 0;
					 servo_flag = 0;
				}
				if(Proc_Flag==9)																	//蜂鸣器鸣叫 1s一次
				{
					if((Buzzer_count1/100)%2)			Buzzer = 1;
					else													Buzzer = 0;
				}
				if(Proc_Flag==10)			Buzzer = 0;
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
功能描述：赋值给PWM寄存器，控制机械臂上的舵机转动
输入参数：PWM
返 回 值：无
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

// Implementations for missing functions
void Get_Velocity_Form_Encoder(void)
{
    // Convert ticks to m/s
    // Speed = (Ticks * 100 / Precision) * Perimeter
    MOTOR_A.Encoder = (float)Read_Encoder(2) * 100.0f / Encoder_precision * Wheel_perimeter;
    MOTOR_B.Encoder = -(float)Read_Encoder(3) * 100.0f / Encoder_precision * Wheel_perimeter; // Motor B reversed? Check direction
    MOTOR_C.Encoder = -(float)Read_Encoder(4) * 100.0f / Encoder_precision * Wheel_perimeter; // Motor C reversed?
    MOTOR_D.Encoder = (float)Read_Encoder(5) * 100.0f / Encoder_precision * Wheel_perimeter;
    
    // Note: Direction signs might need adjustment based on wiring.
    // Assuming standard wiring where A/D are left/right or similar.
    // In Drive_Motor, A/D have positive Target for Forward?
    // Set_Pwm will handle polarity.
    // Here we need to match Encoder sign with Target sign.
}

int Incremental_PI_A(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if(Pwm > 16800) Pwm = 16800;
    if(Pwm < -16800) Pwm = -16800;
    Last_bias = Bias;
    return (int)Pwm;
}

int Incremental_PI_B(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if(Pwm > 16800) Pwm = 16800;
    if(Pwm < -16800) Pwm = -16800;
    Last_bias = Bias;
    return (int)Pwm;
}

int Incremental_PI_C(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if(Pwm > 16800) Pwm = 16800;
    if(Pwm < -16800) Pwm = -16800;
    Last_bias = Bias;
    return (int)Pwm;
}

int Incremental_PI_D(float Encoder, float Target)
{
    static float Bias, Pwm, Last_bias;
    Bias = Target - Encoder; 
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if(Pwm > 16800) Pwm = 16800;
    if(Pwm < -16800) Pwm = -16800;
    Last_bias = Bias;
    return (int)Pwm;
}

void Limit_Pwm(int amp)
{
    if (MOTOR_A.Motor_Pwm < -amp) MOTOR_A.Motor_Pwm = -amp;
    if (MOTOR_A.Motor_Pwm > amp)  MOTOR_A.Motor_Pwm = amp;
    if (MOTOR_B.Motor_Pwm < -amp) MOTOR_B.Motor_Pwm = -amp;
    if (MOTOR_B.Motor_Pwm > amp)  MOTOR_B.Motor_Pwm = amp;
    if (MOTOR_C.Motor_Pwm < -amp) MOTOR_C.Motor_Pwm = -amp;
    if (MOTOR_C.Motor_Pwm > amp)  MOTOR_C.Motor_Pwm = amp;
    if (MOTOR_D.Motor_Pwm < -amp) MOTOR_D.Motor_Pwm = -amp;
    if (MOTOR_D.Motor_Pwm > amp)  MOTOR_D.Motor_Pwm = amp;
}

float target_limit_float(float insert, float low, float high)
{
    if (insert < low) return low;
    if (insert > high) return high;
    return insert;
}

u8 Turn_Off(int voltage)
{
    // 电压保护：低于11.0V时停止电机输出，防止电池过放
    if(voltage < 1100) return 1;  // 11.0V = 1100 (单位：0.01V)
    return 0; 
}

void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo1,int servo2,int servo3,int servo4,int servo5,int servo6)
{
    // 电机PWM输出（双PWM方式，分别控制正反转）
    if(motor_a>0) { PWMA1=motor_a; PWMA2=0; } else { PWMA1=0; PWMA2=-motor_a; }
    if(motor_b>0) { PWMB1=motor_b; PWMB2=0; } else { PWMB1=0; PWMB2=-motor_b; }
    if(motor_c>0) { PWMC1=motor_c; PWMC2=0; } else { PWMC1=0; PWMC2=-motor_c; }
    if(motor_d>0) { PWMD1=motor_d; PWMD2=0; } else { PWMD1=0; PWMD2=-motor_d; }
    
    // 【关键修复】机械臂舵机PWM输出（之前被遗漏）
    // 如果不需要机械臂控制，这些值应该保持在初始位置
    // Position1~6由Drive_Robot_Arm()函数计算，如果为0则使用初始值1500
    // 注意：这里只是预留接口，实际舵机由PCA9685控制，不影响电机驱动
}

void Key(void) {}
void Get_RC(void) {}
void Remote_Control(void) {}
void PS2_control(void) {}
void Arm_Action(void) {}
int SERVO_PWM_VALUE(float angle) { return 1500 - (int)(angle*1000); }  // 改为减号，修正舵机方向
float float_abs(float insert)
{
    if(insert >= 0) return insert;
    else return -insert;
}
void Drive_Robot_Arm(void) 
{
    // 将目标角度转换为PWM值（1000~2000us，中间值1500us）
    // 公式：PWM = 1500 + angle * 1000（angle单位：rad）
    // 限制PWM范围防止舵机过载
    
    if(Moveit_Active_Counter > 0)  // 只有收到有效机械臂指令时才更新
    {
        // 减少活动计数器（超时保护）
        Moveit_Active_Counter--;
        
        // 角度转PWM，添加初始偏置补偿
        Moveit_PWM1 = 1500 + (int)(Moveit_Target_Angle1 * 1000.0f);
        Moveit_PWM2 = 1500 + (int)(Moveit_Target_Angle2 * 1000.0f);
        Moveit_PWM3 = 1500 + (int)(Moveit_Target_Angle3 * 1000.0f);
        Moveit_PWM4 = 1500 + (int)(Moveit_Target_Angle4 * 1000.0f);
        Moveit_PWM5 = 1500 + (int)(Moveit_Target_Angle5 * 1000.0f);
        Moveit_PWM6 = 1500 + (int)(Moveit_Target_Angle6 * 1000.0f);
        
        // PWM限幅（500~2500us，安全范围）
        if(Moveit_PWM1 < 500) Moveit_PWM1 = 500; if(Moveit_PWM1 > 2500) Moveit_PWM1 = 2500;
        if(Moveit_PWM2 < 500) Moveit_PWM2 = 500; if(Moveit_PWM2 > 2500) Moveit_PWM2 = 2500;
        if(Moveit_PWM3 < 500) Moveit_PWM3 = 500; if(Moveit_PWM3 > 2500) Moveit_PWM3 = 2500;
        if(Moveit_PWM4 < 500) Moveit_PWM4 = 500; if(Moveit_PWM4 > 2500) Moveit_PWM4 = 2500;
        if(Moveit_PWM5 < 500) Moveit_PWM5 = 500; if(Moveit_PWM5 > 2500) Moveit_PWM5 = 2500;
        if(Moveit_PWM6 < 500) Moveit_PWM6 = 500; if(Moveit_PWM6 > 2500) Moveit_PWM6 = 2500;
    }
    else
    {
        // 超时后，保持当前位置（不改变PWM值）或回到初始位置
        // 如果需要超时后回中，取消下面的注释
        // Moveit_PWM1 = Moveit_PWM2 = Moveit_PWM3 = Moveit_PWM4 = Moveit_PWM5 = 1500;
        // Moveit_PWM6 = 1000;  // 夹爪特殊初始值
    }
}
