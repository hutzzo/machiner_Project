#include "usartx.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;


/**************************************************************************
Function: Usartx3, Usartx1,Usartx5 and CAN send data task 
Input   : none
Output  : none
�������ܣ�����3������1������5��CAN������������
��ڲ�������
����  ֵ����
**************************************************************************/
void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 20hz
			//��������20Hz��Ƶ������
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));
			if(Check==0)
			{
				//Assign the data to be sent
				//��Ҫ���з��͵����ݽ��и�ֵ
				data_transition(); 
				USART1_SEND();     //Serial port 1 sends data //����1��������
				USART3_SEND();     //Serial port 3 (ROS) sends data  //����3(ROS)��������
				USART5_SEND();		 //Serial port 5 sends data //����5��������
				CAN_SEND();        //CAN send data //CAN��������	
			}
		}
}
/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER_CAR; //Frame_header //֡ͷ
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL_CAR;     //Frame_tail //֡β
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//���ݲ�ͬ����ѡ��ͬ�˶�ѧ�㷨�����˶�ѧ���⣬�Ӹ������ٶ���������ٶ�
	switch(Car_Mode)
	{	
		case Mec_Car:      
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000;
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder-MOTOR_D.Encoder)/4)*1000; 
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4/(Axle_spacing+Wheel_spacing))*1000;         
		  break; 
		
		case FourWheel_Car:
      Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000; 
	    Send_Data.Sensor_Str.Y_speed = 0;
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.Encoder-MOTOR_A.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/2/(Axle_spacing+Wheel_spacing))*1000;
		 break; 
		
		case Tank_Car:   
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; 
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/(Wheel_spacing)*1000);
			break; 
	}
	
	//The acceleration of the triaxial acceleration //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Accelerometer.X_data= imu.accel.y; //The accelerometer Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Accelerometer.Y_data=-imu.accel.x; //The accelerometer X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	Send_Data.Sensor_Str.Accelerometer.Z_data= imu.accel.z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //���ٶȼ�Z��ת����ROS����Z��
	
	//The Angle velocity of the triaxial velocity //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Gyroscope.X_data= imu.gyro.y; //The Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Gyroscope.Y_data=-imu.gyro.x; //The X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //����������λʹ��״̬����ô��������Z����ٶ�
		Send_Data.Sensor_Str.Gyroscope.Z_data=imu.gyro.z;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //֡ͷ
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //С������ʧ�ܱ�־λ
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//С�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//��ص�ѹ,���Ϊ����8λ���ݷ���
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //����У��λ���㣬ģʽ1�Ƿ�������У��
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //֡β
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : none
Output  : none
�������ܣ�����3��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart3_send(Send_Data.buffer[i]);
	}	 
}

void USART3_Return(void)
{
    int i;
    for(i=0; i<message_count; i++)
    {
        usart3_send(uart3_receive_message[i]);
    }
    usart3_send('\r');
    usart3_send('\n');
}
void USART2_Return(void)
{
	  int i;
    printf("{#");
    
    for(i=0; i<app_count; i++)
    {
        printf("%c",uart2_receive_message[i]);
    }
    printf("}$");
    printf("\r\n");
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : none
Output  : none
�������ܣ�����5��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART5_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart5_send(Send_Data.buffer[i]);
	}	 
}
/**************************************************************************
Function: CAN sends data
Input   : none
Output  : none
�������ܣ�CAN��������
��ڲ�������
�� �� ֵ����
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
}
/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //ʹ�ܴ���1
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //��ʼ������2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //ʹ�ܴ���2 
}
/**************************************************************************
Function: Serial port 3 initialization
Input   : none
Output  : none
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);      //Initialize serial port 3 //��ʼ������3
	
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                     //Enable serial port 3 //ʹ�ܴ���3 
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
�������ܣ�����5��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
		//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
�������ܣ�����1�����ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		u8 Usart_Receive;
		static u8 Count_car,Count_moveit;
		static u8 rxbuf_car[11],rxbuf_moveit[16];
		int check=0,error=1,i;
		
		Usart_Receive = USART_ReceiveData(USART1); //Read the data //��ȡ����
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //����10��ǰ����������
			return 0;	
			
		//��ȡ���������ݣ���ʼУ�飬У��ɹ���ֵXYZĿ���ٶ�
    rxbuf_car[Count_car]=Usart_Receive;  
		rxbuf_moveit[Count_moveit]=Usart_Receive;
		
		//ȷ�������һ������ΪFRAME_HEADER
    if(Usart_Receive == FRAME_HEADER_CAR||Count_car>0) 		Count_car++; else Count_car=0; 
		if(Usart_Receive == FRAME_HEADER_MOVEIT||Count_moveit>0) Count_moveit++; else Count_moveit=0; 
		
		if (Count_car== 11) //Verify the length of the packet //��֤���ݰ��ĳ���
		{   
				Count_car =0; //Prepare for the serial port data to be refill into the array //Ϊ����������������������׼��
				if(rxbuf_car[10] == FRAME_TAIL_CAR) //Verify the frame tail of the packet //��֤���ݰ���֡β
				{			
					for(i=0; i<9; i++)
					{
						//XOR bit check, used to detect data error
						//���λУ�飬���ڼ�������Ƿ����
						check=rxbuf_car[i]^check; 
					}
					if(check==rxbuf_car[9]) 
						//XOR bit check successful
					  //���λУ��ɹ�
					  error=0; 
					
					if(error==0)	 
				  {		
            if(Usart1_ON_Flag==0)
						{	
							//Serial port 1 controls flag position 1, other flag position 0
							//����1���Ʊ�־λ��1��������־λ��0
							Usart1_ON_Flag=1;
							Usart5_ON_Flag=0;
							APP_ON_Flag=0;
							PS2_ON_Flag=0;
							Remote_ON_Flag=0;
							CAN_ON_Flag=0;

						}		
							command_lost_count=0; //CAN/���ڿ������ʧ��������
		
						//Calculate the 3-axis target velocity from the serial data, which is divided into 8-bit high and 8-bit low units mm/s
						//�Ӵ�������������Ŀ���ٶȣ��ָ�8λ�͵�8λ ��λmm/s
						Move_X=(short)((rxbuf_car[3]<<8)+(rxbuf_car[4])); 
						Move_Y=(short)((rxbuf_car[5]<<8)+(rxbuf_car[6])); 
						Move_Z=(short)((rxbuf_car[7]<<8)+(rxbuf_car[8])); 
						
						//Unit conversion, mm/s->m/s
						//��λת��, mm/s->m/s
						Move_X=Move_X/1000; 
						Move_Y=Move_Y/1000; 
						Move_Z=Move_Z/1000; 
					}
			  }
		 }
		
		  if (Count_moveit == 16)	//��֤���ݰ��ĳ���
		{   
				Count_moveit=0;//���¿�ʼ����
				if(rxbuf_moveit[15] == FRAME_TAIL_MOVEIT) //��֤���ݰ���β��У����Ϣ
				{
					
					for(i=0; i<14; i++)
					{
						check=rxbuf_moveit[i]^check; //������ڼ�������Ƿ����
					}
					if(check==rxbuf_moveit[14]) error=0; //����ɹ�
					
					if(error==0)	 //����У��λ����
				  {		
						Moveit_Angle1=(short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle2=(short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle3=(short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle4=(short)((rxbuf_moveit[7]<<8)+(rxbuf_moveit[8])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle5=(short)((rxbuf_moveit[9]<<8)+(rxbuf_moveit[10])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle6=(short)((rxbuf_moveit[11]<<8)+(rxbuf_moveit[12])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						
						
						Moveit_Angle1=Moveit_Angle1*0.001f; //��λ(����)
						Moveit_Angle2=Moveit_Angle2*0.001f; 
						Moveit_Angle3=Moveit_Angle3*0.001f; 
						Moveit_Angle4=Moveit_Angle4*0.001f;
						Moveit_Angle5=Moveit_Angle5*0.001f; 
						Moveit_Angle6=Moveit_Angle6*0.001f;
						
            if(rxbuf_moveit[13]==follower)    Position_KP=0.4,Position_KI=0,Position_KD=0.5;  //���湦��ʱ��PID�����������Ӧ��������Ϊ����������
						else                              Position_KP=2,Position_KI=0,Position_KD=1;  //Ĭ��ʹ�õ�PID���������ЧӦ�Ͽ죩									
					}
			  }
		 }
	}
		return 0;	
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART2_IRQHandler(void)
{	
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{	      
		static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
		static float Data;
				
		Usart_Receive=USART2->DR; //Read the data //��ȡ����
		
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //����10��ǰ����������
		  return 0;	
		if(Check==0)
		{
		if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&APP_ON_Flag==0)
			//10 seconds after startup, press the forward button of APP to enter APP control mode
		  //The APP controls the flag position 1 and the other flag position 0
			//����10��֮�󣬰���APP��ǰ��������APP����ģʽ
		  //APP���Ʊ�־λ��1��������־λ��0
				PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=1,CAN_ON_Flag=0,Usart1_ON_Flag=0,Usart5_ON_Flag=0;
			  Last_Usart_Receive=Usart_Receive;	
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //����APPת����ƽ���
			Turn_Flag=1;  
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//����APP������ƽ���	
			Turn_Flag=0;	
			
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APPҡ�˿��ƽ�������
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APPת����ƽ�������
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //����ת
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //����ת
			else                         Flag_Left=0,Flag_Right=0,Flag_Direction=0;
//			if     (Usart_Receive==0x41) car_A_steer_flag=1;//С������
//			if     (Usart_Receive==0x45)	car_A_steer_flag=2;//�������
//			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //���ٰ�����+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //���ٰ�����-100mm/s
	  
		if(RC_Velocity<50)   RC_Velocity=50;
		else if(RC_Velocity>1200)  RC_Velocity=1200;
	 // The following is the communication with the APP debugging interface
	 //��������APP���Խ���ͨѶ
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP����ָ����ʼλ
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP����ָ��ֹͣλ

	 if(Flag_PID==1) //Collect data //�ɼ�����
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //��������
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else  if(Receive[1]!=0x23) 
      {								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				switch(Receive[1])
				 {
					 case 0x30:  Moveit_Angle1=Data*0.01f-1.57f;break; //*0.01����Ϊ�����ڷ���ǰ�ǷŴ���һ�ٱ��ģ�������Ҫ����ԭ
					 case 0x31:  Moveit_Angle2=Data*0.01f-1.57f;break; //-1.57����Ϊ�����ڷ���ǰ����1.57��������Ҫ����ԭ
					 case 0x32:  Moveit_Angle3=Data*0.01f-1.57f;break;
					 case 0x33:  Moveit_Angle4=Data*0.01f-1.57f;break;
					 case 0x34:  Moveit_Angle5=Data*0.01f-1.57f;break;
					 case 0x35:  Moveit_Angle6=Data*0.01f-1.57f;break;
					 case 0x36:  RC_Velocity=Data;break;
					 case 0x37:  Velocity_KP=Data;break;
					 case 0x38:  Velocity_KI=Data;break;
				 }
      }		
      //Relevant flag position is cleared			
      //��ر�־λ����			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//��������
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 	 
  }
		else if(Check==1)
	{
		if(Usart_Receive==0x41)					uart2_send_flag = 1;
		else if(Usart_Receive==0x43)		uart2_send_flag = 2;
		else if(Usart_Receive==0x45)		uart2_send_flag = 3;
		else if(Usart_Receive==0x47)		uart2_send_flag = 4;
		else if(Usart_Receive==0x5A)		uart2_send_flag = 5;
		
		
//			uart2_receive_message[app_count] = Usart_Receive;
//			if(Usart_Receive=='#')		
//			{
//				uart2_receive_message[app_count] = '\0';
//				uart2_send_flag = 1;
//			}
//			app_count++;
	}
}
  return 0;	
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{	
	static u8 Count_car,Count_moveit;
	static u8 rxbuf_moveit[16];
	u8 Usart_Receive;
	int check=0,error=1,i;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		Usart_Receive = USART_ReceiveData(USART3);//Read the data //��ȡ����
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //����10��ǰ����������
		  return 0;	
		if(Check==0)
		{
		//Fill the array with serial data
		//����������������
    Receive_Data.buffer[Count_car]=Usart_Receive;
		rxbuf_moveit[Count_moveit]=Usart_Receive;
		
		
		// Ensure that the first data in the array is FRAME_HEADER
		//ȷ�������һ������ΪFRAME_HEADER
		if(Usart_Receive == FRAME_HEADER_CAR||Count_car>0) Count_car++; else Count_car=0; //���տ��Ƶ��̵�����֡
		if(Usart_Receive == FRAME_HEADER_MOVEIT||Count_moveit>0) Count_moveit++; else Count_moveit=0; //���տ��ƻ�е�۵�����֡
		if (Count_car == 11) //Verify the length of the packet //��֤���ݰ��ĳ���
		{   
				Count_car=0; //Prepare for the serial port data to be refill into the array //Ϊ����������������������׼��
				if(Receive_Data.buffer[10] == FRAME_TAIL_CAR) //Verify the frame tail of the packet //��֤���ݰ���֡β
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//�������λУ����㣬ģʽ0�Ƿ�������У��
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				  {		
						//All modes flag position 0, USART3 control mode
            //����ģʽ��־λ��0��ΪUsart3����ģʽ						
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart1_ON_Flag=0;
						Usart5_ON_Flag=0;
						command_lost_count=0; //CAN/���ڿ������ʧ��������

						//Calculate the target speed of three axis from serial data, unit m/s
						//�Ӵ�������������Ŀ���ٶȣ� ��λm/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				  }
					
			}
		}
		  if (Count_moveit == 16)	//��֤���ݰ��ĳ���
		{   
				Count_moveit=0;//���¿�ʼ����
				if(rxbuf_moveit[15] == FRAME_TAIL_MOVEIT) //��֤���ݰ���β��У����Ϣ
				{
					 
					for(i=0; i<14; i++)
					{
						check=rxbuf_moveit[i]^check; //������ڼ�������Ƿ����
					}
					if(check==rxbuf_moveit[14]) error=0; //����ɹ�
					
					if(error==0)	 //����У��λ����
				  {		
						Moveit_Angle1=(short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle2=(short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle3=(short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle4=(short)((rxbuf_moveit[7]<<8)+(rxbuf_moveit[8])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle5=(short)((rxbuf_moveit[9]<<8)+(rxbuf_moveit[10])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle6=(short)((rxbuf_moveit[11]<<8)+(rxbuf_moveit[12])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						
						
						Moveit_Angle1=Moveit_Angle1*0.001f; //��λ(����)
						Moveit_Angle2=Moveit_Angle2*0.001f; 
						Moveit_Angle3=Moveit_Angle3*0.001f; 
						Moveit_Angle4=Moveit_Angle4*0.001f;
						Moveit_Angle5=Moveit_Angle5*0.001f; 
						Moveit_Angle6=Moveit_Angle6*0.001f;
						
            if(rxbuf_moveit[13]==follower)    Position_KP=0.4,Position_KI=0,Position_KD=0.5;  //���湦��ʱ��PID�����������Ӧ��������Ϊ����������
						else                              Position_KP=2,Position_KI=0,Position_KD=1;  //Ĭ��ʹ�õ�PID���������ЧӦ�Ͽ죩					
					}
			  }
		 }
	}
		else if(Check==1)
		{
			uart3_receive_message[message_count] = Usart_Receive;
			if(Usart_Receive=='#')		
			{
				uart3_receive_message[message_count]='\0';
				uart3_send_flag = 1;
			}
			message_count++;
		}
}		
  return 0;	
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART5_IRQHandler(void)
{	
	static u8 Count_car,Count_moveit;
	static u8 rxbuf_moveit[16];
	u8 Usart_Receive;
	int check=0,error=1,i;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
		Usart_Receive = USART_ReceiveData(USART3);//Read the data //��ȡ����
		if(SysVal.Time_count<CONTROL_DELAY)
			// Data is not processed until 10 seconds after startup
		  //����10��ǰ����������
		  return 0;	
		
		//Fill the array with serial data
		//����������������
    Receive_Data.buffer[Count_car]=Usart_Receive;
		rxbuf_moveit[Count_moveit]=Usart_Receive;
		
		
		// Ensure that the first data in the array is FRAME_HEADER
		//ȷ�������һ������ΪFRAME_HEADER
		if(Usart_Receive == FRAME_HEADER_CAR||Count_car>0) Count_car++; else Count_car=0; //���տ��Ƶ��̵�����֡
		if(Usart_Receive == FRAME_HEADER_MOVEIT||Count_moveit>0) Count_moveit++; else Count_moveit=0; //���տ��ƻ�е�۵�����֡
		if (Count_car == 11) //Verify the length of the packet //��֤���ݰ��ĳ���
		{   
				Count_car=0; //Prepare for the serial port data to be refill into the array //Ϊ����������������������׼��
				if(Receive_Data.buffer[10] == FRAME_TAIL_CAR) //Verify the frame tail of the packet //��֤���ݰ���֡β
				{
					//Data exclusionary or bit check calculation, mode 0 is sent data check
					//�������λУ����㣬ģʽ0�Ƿ�������У��
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 
				  {		
						//All modes flag position 0, USART3 control mode
            //����ģʽ��־λ��0��ΪUsart3����ģʽ						
						PS2_ON_Flag=0;
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart1_ON_Flag=0;
						Usart5_ON_Flag=1;
						command_lost_count=0; //CAN/���ڿ������ʧ��������
						
						//Calculate the target speed of three axis from serial data, unit m/s
						//�Ӵ�������������Ŀ���ٶȣ� ��λm/s
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				  }
					
			}
		}
		  if (Count_moveit == 16)	//��֤���ݰ��ĳ���
		{   
				Count_moveit=0;//���¿�ʼ����
				if(rxbuf_moveit[15] == FRAME_TAIL_MOVEIT) //��֤���ݰ���β��У����Ϣ
				{
					 
					for(i=0; i<14; i++)
					{
						check=rxbuf_moveit[i]^check; //������ڼ�������Ƿ����
					}
					if(check==rxbuf_moveit[14]) error=0; //����ɹ�
					
					if(error==0)	 //����У��λ����
				  {		
						Moveit_Angle1=(short)((rxbuf_moveit[1]<<8)+(rxbuf_moveit[2])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle2=(short)((rxbuf_moveit[3]<<8)+(rxbuf_moveit[4])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle3=(short)((rxbuf_moveit[5]<<8)+(rxbuf_moveit[6])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle4=(short)((rxbuf_moveit[7]<<8)+(rxbuf_moveit[8])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle5=(short)((rxbuf_moveit[9]<<8)+(rxbuf_moveit[10])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Moveit_Angle6=(short)((rxbuf_moveit[11]<<8)+(rxbuf_moveit[12])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						
						
						Moveit_Angle1=Moveit_Angle1*0.001f; //��λ(����)
						Moveit_Angle2=Moveit_Angle2*0.001f; 
						Moveit_Angle3=Moveit_Angle3*0.001f; 
						Moveit_Angle4=Moveit_Angle4*0.001f;
						Moveit_Angle5=Moveit_Angle5*0.001f; 
						Moveit_Angle6=Moveit_Angle6*0.001f;
						
            if(rxbuf_moveit[13]==follower)    Position_KP=0.4,Position_KI=0,Position_KD=0.5;  //���湦��ʱ��PID�����������Ӧ��������Ϊ����������
						else                              Position_KP=2,Position_KI=0,Position_KD=1;  //Ĭ��ʹ�õ�PID���������ЧӦ�Ͽ죩						
					}
			  }
		 }
	} 
  return 0;	
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ��������Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz��ת��Ϊ������С������ǰ��ת��
��ڲ�����Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz����λ��m/s��rad/s
����  ֵ��������С������ǰ��ת�ǣ���λ��rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//������С����Ҫ������Сת��뾶
	//���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶��
	//�ᵼ��С���˶�Ħ���������ߣ�����Ӱ�����Ч��
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//����Ŀ����ٶȣ����ǰ���ٶȣ����ת��뾶����Сת��뾶
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//����ת�����м����
	short transition; 
	
	//����8λ�͵�8λ���ϳ�һ��16λ��short������
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //��λת��, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
�������ܣ�����5��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//��Ҫ���͵����ݽ���У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//�Խ��յ������ݽ���У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}






