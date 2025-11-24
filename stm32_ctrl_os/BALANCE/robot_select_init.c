#include "robot_select_init.h"

//Initialize the robot parameter structure
//��ʼ�������˲����ṹ��
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
	//ADCֵ�ֶα�����ȡ����С���ͺ�������Ŀǰ��6��С���ͺţ�CAR_NUMBER=6
	Divisor_Mode=2048/CAR_NUMBER+2;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //Collect the pin information of potentiometer //�ɼ���λ��������Ϣ	
  if(Car_Mode>2)Car_Mode=2;
	Car_Mode=Akm_Car;
	
	switch(Car_Mode)
	{
		case Mec_Car:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     HALL_30F, Hall_13, Mecanum_75);            break; //�����ķ��С��
		case FourWheel_Car: Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0,                     HALL_30F, Hall_13, Black_WheelDiameter);   break; //������ 
		case Tank_Car:      Robot_Init(Tank_wheelSpacing,        0,                        0,                     HALL_30F, Hall_13, Tank_WheelDiameter);    break; //�Ĵ���
		case Akm_Car:       Robot_Init(Akm_wheelspacing,         Akm_axlespacing,          0,                     HALL_30F, Hall_13, Black_WheelDiameter);   break;
	}
	
	
	//Check the parameters//�Լ���ز���
	switch(Car_Mode)
  {
	 case Mec_Car:       CheckPhrase1=8, CheckPhrase2=14; break; //�����ķ��С��
	 case FourWheel_Car: CheckPhrase1=8, CheckPhrase2=11; break; //������ 
	 case Tank_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //�Ĵ���
	 case Akm_Car:       CheckPhrase1=8, CheckPhrase2=11; break;
  }
}

/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ� ������������� ��ֱ̥��
����  ֵ����
**************************************************************************/
void Robot_Init(double wheelspacing, float axlespacing, float omni_turn_radiaus, float gearratio,float Accuracy,float tyre_diameter) // 
{
	//wheelspacing, Mec_Car is half wheelspacing
	//�־� ���ֳ�Ϊ���־�
  Robot_Parament.WheelSpacing=wheelspacing; 
	//axlespacing, Mec_Car is half axlespacing
  //��� ���ֳ�Ϊ�����	
  Robot_Parament.AxleSpacing=axlespacing;   
	//Rotation radius of omnidirectional trolley
  //ȫ����С����ת�뾶		
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus; 
	//motor_gear_ratio
	//������ٱ�
  Robot_Parament.GearRatio=gearratio; 
	//Number_of_encoder_lines
  //����������(����������)	
  Robot_Parament.EncoderAccuracy=Accuracy;
	//Diameter of driving wheel
  //������ֱ��	
  Robot_Parament.WheelDiameter=tyre_diameter;       
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//���(����)ת1Ȧ��Ӧ�ı�������ֵ
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference
  //�������ܳ�	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//wheelspacing, Mec_Car is half wheelspacing
  //�־� ���ֳ�Ϊ���־�  
  Wheel_spacing=Robot_Parament.WheelSpacing; 
  //axlespacing, Mec_Car is half axlespacing	
  //��� ���ֳ�Ϊ�����	
	Axle_spacing=Robot_Parament.AxleSpacing; 
	//Rotation radius of omnidirectional trolley
  //ȫ����С����ת�뾶	
	Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus; 
}


