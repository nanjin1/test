#include "barrier.h"
#include "sys.h"
#include "delay.h"
#include "speed_ctrl.h"
#include "motor.h"
#include "pid.h"
#include "imu_task.h"
#include "scaner.h"
#include "turn.h"
#include "map.h"
#include "motor_task.h"
#include "pid.h"
#include "math.h"
#include "bsp_buzzer.h"
#include "bsp_led.h"
#include "stdio.h"
#include "motor_task.h"
#include "QR.h"
#include "motion.h"
#include "bsp_linefollower.h"
#include "openmv.h"
#include "Rudder_control.h"
#include "encoder.h"
#define BACK_SPEEDM -500
#define BACK_SPEED  -450 //-300
#define BACK_SPEED1 -200
#define GOSPEED 10

extern int BW_num[];
uint8_t color_flag[5]={0};
uint8_t  value;//openmv�ӿ�
uint8_t special_arrive=0;
//������ϵ�к���
//1._shake��ƽ̨ʶ������У���ֹ��һЩ�������ص��³���ǰֹͣ,ͬʱ���ײ���Ŀ��
//2._Rshake�����ж����Ƿ񾭹����˰�
//3._pshakeʶ��������״̬
void Anti_shake(int Uneed_time)
{
	uint32_t i = 0;  //ѭ���������ó�unit32_t����forѭ���������ָ��
	for(i = 0; i < Uneed_time ; i++)
	{
		
		while(Infrared_ahead == 1)
		{
			vTaskDelay(1);
		}
	}
}

int AI_shake(int Uneed_time)
{
//	uint32_t i = 0, cnt = 0;  //ѭ���������ó�unit32_t����forѭ���������ָ��
//	for(i = 0; i < Uneed_time ; i++)
//	{
//		HAL_Delay(1);
//		if(AI == 1)
//		{
//			cnt++;
//		}
//	}
//	if(cnt >= Uneed_time*0.5)  return 1;
	return 0;
}

//void Anti_encoder(int Uneed_time)
//{
//	uint32_t i = 0;  //ѭ���������ó�unit32_t����forѭ���������ָ��
//	for(i = 0; i < Uneed_time ; i++)
//	{
//		HAL_Delay(1);
//		while(read_encoder(2) == 0);
//	}
//}

//void Anti_Rshake()
//{
//	uint32_t i = 0;
//	uint32_t cnt = 0;
//	while(i < 148)
//	{
//		for(i = 0; i < 150 ; i++)
//		{
//			HAL_Delay(1);
//			if(fabs(imu.roll) > 2) cnt++;
//			if(cnt == 30)
//			{
//				i = 0;
//				cnt = 0;
//				break;
//			}
//			//while(fabs(imu.roll) > 2);
//		}
//	}
//}

int Anti_Pshake()
{
	int sum = 0;
	for(uint32_t i = 0; i < 5; i++)
	{
		HAL_Delay(1);
		sum += imu.pitch;
	}
	if(sum/5 > 3) return 1;
	else return 0;
}

/************************************************/

void Stage()		//flag==1ʱȡ���ȽǶȣ�flag==0ʱȡ��ԽǶ�
{
	float num = 0; //distan = 0;
	nodesr.nowNode.flag=0;
//	pid_mode_switch(is_Line);
//    motor_all.Cspeed = 600;
//	struct PID_param origin_param = gyroT_pid_param;
	gyroT_pid_param.kp=40;
	float origin_turnM=motor_all.GyroT_speedMax;
	motor_all.GyroT_speedMax=550;
    Rudder_control(170,0);//��վ����
//	if ((Scaner.detail & 0X0180) == 0X0180) //��������м�λ��
//	{
//		angle.AngleG = getAngleZ();//��ƽ���ߵĽǶ�
//		pid_mode_switch(is_Gyro);
//		motor_all.Gspeed = 400;
//	}
//	else{
//		angle.AngleG = nodesr.nowNode.angle;//��ƽ���ߵĽǶ�
//		pid_mode_switch(is_Gyro);
//		motor_all.Gspeed = 400;
//	}
//	while(imu.pitch<basic_p+4)
//	{
//		vTaskDelay(2);
//	}
//    while(imu.pitch<Up_pitch+5)
//	{
//		vTaskDelay(2);
//		if(Scaner.ledNum<4)
//		{
//			break;
//		}
//	}//��ʼ��ƽ̨
//    while(Scaner.ledNum>8)
//	{
//		vTaskDelay(2);
//	}
//	buzzer_on();
	angle.AngleG=getAngleZ();
	pid_mode_switch(is_Gyro);
    motor_all.Gspeed = 400;
	if (nodesr.nowNode.nodenum == P2)
	{
		Stage_P2();
		return;
	}
	while(imu.pitch>After_up)
	{
		vTaskDelay(5);
	}//������
   
	while(Infrared_ahead == 0)
	{
		vTaskDelay(5);
	}  //ײ����
	
	
//	num = motor_all.Distance;

//	while(motor_all.Distance - num < 20)//ǰ��һ�ξ���
//	{
//		vTaskDelay(5);
//	}

	//delay_ms(300);
	CarBrake();
	
	//HAL_Delay(1000);
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	num = motor_all.Distance;				//����һ�ξ���
	
	pid_mode_switch(is_No);
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	uint8_t time=0;
	while (num-motor_all.Distance < 4) //9
	{
		if(time==0)
		{
			Rudder_control(150,2);//450���ַ���  150����
			vTaskDelay(100);
			Rudder_control(450,2);//450���ַ���  150����
			time=1;
		}
		vTaskDelay(2);
	}
	CarBrake();	
	Rudder_control(400,4);//130���ַ���  400����
	vTaskDelay(100);
	Rudder_control(130,4);//130���ַ���  400����
	vTaskDelay(50);

//	if(nodesr.nowNode.nodenum!=P7&nodesr.nowNode.nodenum!=P8)
//	{
//		while(BW_add==0)
//		{
//			vTaskDelay(2);
//		}//�ȴ�����
//	}
//	if(nodesr.nowNode.nodenum==P1)
//	{
//		BW_num[0]=BW_add;
//	}
//	else if(nodesr.nowNode.nodenum==P3|nodesr.nowNode.nodenum==P4)
//	{
//		BW_num[1]=BW_add;
//	}
//	else if(nodesr.nowNode.nodenum==P5|nodesr.nowNode.nodenum==P6)
//	{
//		BW_num[2]=BW_add;
//	}
//	BW_add=0;
	BW_num[0] = 3;
	BW_num[1] = 5;
	BW_num[2] = 8;
	motor_pid_clear();
    if(nodesr.nowNode.nodenum==P1)
	{
		bofang_zhiding(2);
	}
	else if(nodesr.nowNode.nodenum==P3)
	{//ƽ̨4
		bofang_zhiding(3);
	}
	else if(nodesr.nowNode.nodenum==P4)
	{
		bofang_zhiding(4);
	}
	else if(nodesr.nowNode.nodenum==P5)
	{
		bofang_zhiding(6);
	}
	else if(nodesr.nowNode.nodenum==P6)
	{
		bofang_zhiding(5);
	}
	else if(nodesr.nowNode.nodenum==P7)
	{
		bofang_zhiding(8);
	}
	else if(nodesr.nowNode.nodenum==P8)
	{
		bofang_zhiding(7);
	}
	Turn_Angle_Relative(179);
	while (fabs(angle.AngleT-getAngleZ())>2)
	{
		vTaskDelay(5);
	}
	motor_pid_clear();
	if(check_BW(nodesr.nowNode.nodenum))//���ֱ��صĶ���
	{
		Rudder_control(400,4);//130���ַ���  400����
        Rudder_control(150,2);//450���ַ���  150����
		bofang_zhiding(10);
		Turn_Angle360();
		Rudder_control(130,4);//130���ַ���  400����
        Rudder_control(450,2);//450���ַ���  150����
	}
	motor_pid_clear();
    pid_mode_switch(is_Gyro);
	motor_all.Gspeed=500;
	angle.AngleG=nodesr.nowNode.angle-179;
	while(imu.pitch>Down_pitch)
	{
		vTaskDelay(2);
	}
	motor_pid_clear();
    Rudder_control(320,0);//������
	pid_mode_switch(is_Line);
	motor_all.Cspeed=800;
	nodesr.nowNode.function=0;	//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}
//������
void Special_Node()
{
	
	if(((nodesr.nowNode.flag&DRIGHT)==DRIGHT)&((nodesr.nowNode.flag&CRIGHT)==CRIGHT)&(nodesr.nowNode.nodenum==N5))//N4-N5��ѭ�� ��ѭ�� ��Ե����
	{//N4-N5
		nodesr.nowNode.flag&=(~RIGHT_LINE);//ȡ����ѭ����־λ
		nodesr.nowNode.flag|=LEFT_LINE;//��ѭ��
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}//�ҷֲ�
		nodesr.nowNode.flag&=(~CRIGHT);//ȡ���ҷֲ��־λ
	    while(deal_arrive()!=1)//�Ұ����
		{
			vTaskDelay(2);
			scaner_set.EdgeIgnore=6;
			special_arrive=1;
		}
	}
	else if((nodesr.nowNode.nodenum==N4)&((nodesr.nowNode.flag&CRIGHT)==CRIGHT))//N5-N4
	{
		float num=0;
		nodesr.nowNode.flag&=(~RIGHT_LINE);
		nodesr.nowNode.flag|=LEFT_LINE;//��ѭ��
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}
		num=motor_all.Distance;
		while(motor_all.Distance-num<10)//��һ����ֲ�·������10����
		{
			vTaskDelay(2);
		}
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}
	}
	else if((nodesr.nowNode.nodenum==N4)&((nodesr.nowNode.flag&CLEFT)==CLEFT))//N3-N4
	{
		float num=0;
		nodesr.nowNode.flag&=(~LEFT_LINE);
		nodesr.nowNode.flag|=RIGHT_LINE;//��ѭ��
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}
		num=motor_all.Distance;
		while(motor_all.Distance-num<10)//��һ����ֲ�·������10����
		{
			vTaskDelay(2);
		}
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}
	}
	else if(((nodesr.nowNode.nodenum==N13)&((nodesr.nowNode.flag&CLEFT)==CLEFT))&(((nodesr.nowNode.flag&CRIGHT)==CRIGHT)&((nodesr.nowNode.flag&DLEFT)==DLEFT))&((nodesr.nowNode.flag&DRIGHT)==DRIGHT))
/*P6-N13*/	{
		angle.AngleG=getAngleZ();
		motor_all.Gspeed=300;
		pid_mode_switch(is_Gyro);
	}
//	else if(nodesr.nowNode.nodenum==N9&(nodesr.nowNode.flag&CLEFT)==CLEFT)
//	{
//		float num=0;
//		while(!deal_arrive())
//		{
//			vTaskDelay(2);
//		}
//		num=motor_all.Distance;
//		while(motor_all.Distance-num<65)
//		{
//			vTaskDelay(2);
//		}
//		while(!deal_arrive())
//		{
//			vTaskDelay(2);
//		}
//	}
	else
	{
		angle.AngleG=nodesr.nowNode.angle;
		motor_all.Gspeed=1000;
		pid_mode_switch(is_Gyro);
	}
//	if(((nodesr.nowNode.flag&CRIGHT)==CRIGHT)&((nodesr.nowNode.flag&CLEFT)==CLEFT))//N5-N6  P4-N6����ѭ������ѭ�� 
//	{
//		angle.AngleT=getAngleZ();
//		pid_mode_switch(is_Gyro);
////		nodesr.nowNode.flag|=LEFT_LINE;//��ѭ��
////		while(deal_arrive()!=1)
////		{
////			vTaskDelay(2);
////		}//��⵽�ҷֲ�
////		nodesr.nowNode.flag&=(~CRIGHT);//ȡ���ҷֲ��־λ
////		while(deal_arrive()!=1)
////		{
////			vTaskDelay(2);
////		}//��⵽��ֲ�
////		nodesr.nowNode.flag&=(~LEFT_LINE);//ȡ����ѭ��
////		nodesr.nowNode.flag|=RIGHT_LINE;//��ѭ��
////		special_arrive=1;
//	}
}

//ƽ̨���Ķ���
void Stage_P2()		//flag==1ʱȡ���ȽǶȣ�flag==0ʱȡ��ԽǶ�
{

	while(imu.pitch<basic_p+12)
	{
		vTaskDelay(2);
	}
	while(imu.pitch>basic_p+3)
	{
		vTaskDelay(2);
	}
	float num = motor_all.Distance;
	while(motor_all.Distance-num<15)
	{
		vTaskDelay(2);
	}
	Turn_Angle_Relative(181);		//ת>=180�ȣ�
	while (fabs(angle.AngleT-getAngleZ())>2)
	{
		vTaskDelay(5);
	}
	nodesr.nowNode.function=0;	//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}

/***************************************************
������
************************************************/

void Barrier_Bridge(float step,float speed)	//������
{
	float num=0;
	buzzer_on();
	num=motor_all.Distance;
	motor_all.Gspeed = 800;    //��ƽ���ٶ�
	float now_angle=0;

	angle.AngleG = getAngleZ();//��ƽ���ߵĽǶ�
	pid_mode_switch(is_Gyro);
	now_angle=getAngleZ();

	struct PID_param origin_param = gyroG_pid_param;
	gyroG_pid_param.kp = 1.8;//ԭ��3.7 4.8 7.8 �������7.8 6.8������	
	gyroG_pid_param.ki=0;
	while(imu.pitch <= Up_pitch)	//����ƽ��,��ѭ������������
	{
		vTaskDelay(2);
	}
	//is_Up = false;
	motor_all.Gspeed = 800; 
    while(imu.pitch>After_up)
	{
		buzzer_on();
//		infrare_open=1;
//		if(infrared.outside_right)
//		{
//			angle.AngleG = getAngleZ() + 3;
//		}
//		else if(infrared.outside_left)
//		{
//				angle.AngleG = getAngleZ() - 3; 
//		}
//		else{
//			angle.AngleG = getAngleZ();//��ȷ�ĽǶ�
//		}
		vTaskDelay(2);
	}
	motor_all.Gspeed = 1000; 
	num=motor_all.Distance;
	while(imu.pitch > Down_pitch)          
	{   
		buzzer_on();
		infrare_open=1;
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)==0&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)==1)
		{
			angle.AngleG = now_angle +2;
		}
		else if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)==1&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10)==0)
		{
			angle.AngleG = now_angle -2;
		}
		else
		{
			angle.AngleG = now_angle;
		}
//		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)==0)
//		{
//			angle.AngleG =now_angle + 2;
//		}
//		else if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==0)
//		{
//				angle.AngleG =now_angle - 2; 
//		}
		vTaskDelay(5);
		if(motor_all.Distance-num>70)
			motor_all.Gspeed=500;
	}
	infrare_open=0;
//	while(imu.pitch<Down_pitch)
//	{
//		angle.AngleG = nodesr.nowNode.angle;
//		motor_all.Cspeed = 300;
//		vTaskDelay(2);
//	}//������
	angle.AngleG=now_angle;
//	pid_mode_switch(is_Free);
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);	
//		TIM1->CCR4 = 0; TIM1->CCR2 = 5000;
//			TIM1->CCR1 = 0; TIM1->CCR3 = 5000;
//			TIM2->CCR4 = 0; TIM2->CCR2 = 5000;
//			TIM2->CCR1 = 0; TIM2->CCR3 = 5000;
	}//��ѭ��������ƽ��
	pid_mode_switch(is_Line);
	buzzer_off();
		//·�̼�¼����
	motor_all.Distance = 0;
	motor_all.encoder_avg = 0;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04;  //����·��
	gyroG_pid_param=origin_param;
}

//��¥��
void Barrier_Hill(uint8_t order)  //¥������
{
	float num=0;
	angle.AngleG = getAngleZ();
	buzzer_on();
	struct PID_param origin_param = line_pid_param;
	line_pid_param.kp=45;
	motor_all.Cspeed = 250;   //ԭ����400
	pid_mode_switch(is_Line);
	while(imu.pitch<basic_p+10)
	{
		vTaskDelay(2);
	}
    motor_all.Cspeed = 400; 	
//	while ( imu.pitch >10+basic_p )	
//	{
//		vTaskDelay(2);
//	}//��ʼ��¥��
	while(imu.pitch>-10+basic_p)
	{
		vTaskDelay(2);
	}//��ʼ��¥��
	num=motor_all.Distance;
    while(imu.pitch<basic_p-6)
	{
		vTaskDelay(2);
		if(motor_all.Distance-num>20)
			break;
	}//����¥��
    buzzer_off();
	line_pid_param = origin_param;
	nodesr.nowNode.function=0;//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}


void Sword_Mountain()
{
float num;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	num = motor_all.Distance;
	motor_all.Cspeed = 600; 
	line_pid_param.kp = 50;
	
	
	//mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	while(motor_all.Distance - num < 25)//ǿ����ѭ��λ��
	{
		vTaskDelay(2);
	}
	angle.AngleG = getAngleZ();
	
	gyroG_pid_param.kp = 2.4;   //����ɽ����ƽ��kp
	motor_all.Gspeed = 600;
	pid_mode_switch(is_Gyro);
    num=motor_all.Distance;
	while(imu.pitch < After_up)//��ѭ���ϵ�ɽ
	{
		vTaskDelay(2);
		if(motor_all.Distance-num>25)
			break;
	}
	buzzer_on();
	

	while(imu.pitch > After_down)//��ѭ���µ�ɽ
	{
		vTaskDelay(2);
	}
	gyroG_pid_param = origin_param1;
	line_pid_param = origin_param;
	buzzer_off();
	
	 
	nodesr.nowNode.function=0;//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}

/*****************************************************************************
����壬���������
*****************************************************************************/
void Barrier_HighMountain(float speed)
{
	float origin_turnM=motor_all.GyroT_speedMax;
	float num = 0;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1=gyroG_pid_param;
	line_pid_param.kp = 25;
	line_pid_param.kd = 15;
	gyroG_pid_param.kp=10;
	pid_mode_switch(is_Gyro);
	angle.AngleG=getAngleZ();
	motor_all.Gspeed=speed;
	while(imu.pitch<Up_pitch)
	{
		vTaskDelay(2);
	}//������
	motor_all.Cspeed = speed; //�ʼ��û���µ��ٶ� 90
	motor_all.Cincrement = 20;
	pid_mode_switch(is_Line);
    num=motor_all.Distance;
	motor_all.Cincrement = 20; //���µ��ٶ�  2
	while(motor_all.Distance-num<70)//������ѭ��
	{
		vTaskDelay(2);
	}
	motor_all.Gspeed = speed; //��һ��ƽ����ƽ��	
	angle.AngleG=getAngleZ();
    pid_mode_switch(is_Gyro);
    buzzer_on();
	while(imu.pitch>After_up)
	{
		vTaskDelay(2);
	}
	while(imu.pitch<Up_pitch)
	{
		vTaskDelay(2);
	}//�ڶ�����
	buzzer_off();
	
	Rudder_control(170,0);//320���� 170վ����
	motor_all.Cspeed = speed; 
	motor_all.Cincrement = 20;
	pid_mode_switch(is_Line);
	num=motor_all.Distance;
	while(motor_all.Distance-num<80)//������ѭ��
	{
		vTaskDelay(2);
	}
	motor_all.Gspeed = speed; //�ڶ�����ƽ��	
	angle.AngleG=getAngleZ();
    pid_mode_switch(is_Gyro);
	buzzer_on();
    while(imu.pitch>After_up)
	{
		vTaskDelay(2);
	}//�ڶ�����ƽ̨
	buzzer_off();
	while(Infrared_ahead == 0)
	{
		vTaskDelay(5);
	}  //ײ����
	num = motor_all.Distance;				
	while(motor_all.Distance - num < 20)//ǰ��һ�ξ���
	{
		vTaskDelay(5);
	}
	CarBrake();
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
   	
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED1;
	uint8_t time=0;
	while (num-motor_all.Distance < 9)
	{
		if(time==0)
		{
			Rudder_control(150,2);//450���ַ���  150����
			vTaskDelay(100);
			Rudder_control(450,2);//450���ַ���  150����
			time=1;
		}
		vTaskDelay(2);
	}
	CarBrake();	
	Rudder_control(400,4);//130���ַ���  400����
	vTaskDelay(100);
	Rudder_control(130,4);//130���ַ���  400����
	vTaskDelay(50);
	motor_all.GyroT_speedMax=500;
	bofang_zhiding(8);
	Turn_Angle_Relative(179);//ת180
	while(fabs(angle.AngleT - getAngleZ())>2) //�ж����
	{
		vTaskDelay(2);
	}
	 motor_pid_clear();
//	CarBrake();
//   vTaskDelay(300);
	if(check_BW(nodesr.nowNode.nodenum))//���ֱ��صĶ���
	{
		motor_pid_clear();
		Rudder_control(400,4);//130���ַ���  400����
        Rudder_control(150,2);//450���ַ���  150����
		bofang_zhiding(10);
		Turn_Angle360();
		Rudder_control(130,4);//130���ַ���  400����
        Rudder_control(450,2);//450���ַ���  150����
		motor_pid_clear();
		CarBrake();
		vTaskDelay(300);
	}
	Rudder_control(320,0);
	
	pid_mode_switch(is_Gyro);
	motor_all.Gspeed=400;
	angle.AngleG=getAngleZ();
    
    Barrier_Down_HighMountain(600);

	line_pid_param = origin_param;
	gyroG_pid_param=origin_param1;
    motor_all.GyroT_speedMax=origin_turnM;
	nodesr.nowNode.function=0;//����ϰ���־
	nodesr.flag|=0x04;	//����·��
}

/*****************************************************************************
�������ܣ�����壬��ת��180�����
*****************************************************************************/
void Barrier_Down_HighMountain(float speed)
{
	float num=0;
	line_pid_param.kp = 40;
	while(Scaner.ledNum<=4)
	{
		getline_error();
		vTaskDelay(2);
	}
	while(Scaner.ledNum>=4)
	{
		getline_error();
		vTaskDelay(2);
	}
	CarBrake();
	vTaskDelay(200);
	{//��������
		getline_error();
	    if(Scaner.detail&0xf)
		{
			pid_mode_switch(is_No);
			num=motor_all.Distance;
			motor_all.Lspeed=motor_all.Rspeed=-200;
			while(num-motor_all.Distance<6)
			{
				vTaskDelay(2);
			}
			Turn_Angle_Relative(-15);//160
			while(fabs(angle.AngleT - getAngleZ())>2)//4
			{
				vTaskDelay(2);
			}
			pid_mode_switch(is_Gyro);
			angle.AngleG=getAngleZ();
			motor_all.Gspeed=400;
			while(Scaner.ledNum<=4)
			{
				getline_error();
				vTaskDelay(2);
			}
			while(Scaner.ledNum>=4)
			{
				getline_error();
				vTaskDelay(2);
			}
		}
		else if(Scaner.detail&0xf000)
		{
			pid_mode_switch(is_No);
			num=motor_all.Distance;
			motor_all.Lspeed=motor_all.Rspeed=-200;
			while(num-motor_all.Distance<6)
			{
				vTaskDelay(2);
			}
			Turn_Angle_Relative(15);//160
			while(fabs(angle.AngleT - getAngleZ())>2)//4
			{
				vTaskDelay(2);
			}
			pid_mode_switch(is_Gyro);
			angle.AngleG=getAngleZ();
			motor_all.Gspeed=400;
			while(Scaner.ledNum<=4)
			{
				getline_error();
				vTaskDelay(2);
			}
			while(Scaner.ledNum>=4)
			{
				getline_error();
				vTaskDelay(2);
			}
		}
	}
	pid_mode_switch(is_Line);//�յ����߿�ѭ��
	motor_all.Cspeed=450;
//	pid_mode_switch(is_Gyro);
//	angle.AngleG=getAngleZ();
//	motor_all.Gspeed=450;
	while(imu.pitch>Down_pitch)
	{
		vTaskDelay(2);
	}//����
	
	num=motor_all.Distance;
	while(motor_all.Distance-num<40)
	{
		vTaskDelay(2);
	}//������
	motor_all.Cspeed=600;
	angle.AngleG=getAngleZ();
//	num=motor_all.Distance;
	while(motor_all.Distance-num<70)
	{
		vTaskDelay(2);
	}
	pid_mode_switch(is_Gyro);
	motor_all.Gspeed=450;
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);
	}//��һ��ƽ��
	num=motor_all.Distance;
   while(motor_all.Distance-num<12)
   {
	   vTaskDelay(2);
   }
   buzzer_on();
	pid_mode_switch(is_Line);
	motor_all.Cspeed=speed;
	num=motor_all.Distance;
	while(imu.pitch>Down_pitch)
	{
		vTaskDelay(2);
	}//����
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);
	}//�ڵ���
	buzzer_off();
}


void view()//�򾰵�	
{	
	float num = 0;
	while(Infrared_ahead == 0)		//ײ����
	{
		vTaskDelay(5);
	}
	
	CarBrake();
	num=motor_all.Distance;
	pid_mode_switch(is_No);
	
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	
	while(num-motor_all.Distance<5)
	{
		vTaskDelay(5);
	}
	
	CarBrake();
	
	Turn_Angle_Relative(181);
	while(fabs(angle.AngleT-getAngleZ())>5)
	{
		vTaskDelay(2);
	}
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 30;
	nodesr.nowNode.function=0;
	encoder_clear();
	nodesr.flag|=0x04;	//����·��
}

void view1()//�򾰵�	
{	
	pid_mode_switch(is_Line);
	while(Infrared_ahead == 0);		//ײ����
	delay_ms(100);
	//mpuZreset(gyro.yaw,nodesr.nowNode.angle-10);
	CarBrake();
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}

void back()
{
	pid_mode_switch(is_No);
	
	motor_L0.target = motor_L1.target = BACK_SPEED;
	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	while(infrared.outside_left == 0 && infrared.outside_right == 0);
	CarBrake();
	//ת���ԽǶ�
	angle.AngleT = nodesr.nextNode.angle;
	pid_mode_switch(is_Turn);
	while(fabs(angle.AngleT - getAngleZ())>5);
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	//motor_all.Cspeed=15;
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}
/*****************************************************************************
�����˰�
�ٶ�   ��  ���˰峤�ȣ�cm��
������Ҫ�Ƿ�ֹ������ǰ���벨�˰壬Ȼ����ǰ�����������ʧ��
������һ�γ��ȣ���֤�ܽ��벨�˰壬�ұ�����ǰ���г��Ѿ���ȥ���˰塣
*****************************************************************************/
void Barrier_WavedPlate(float lenght)//���˰峤��
{
	buzzer_on();
	float num = 0;
    struct PID_param origin_param = line_pid_param;
	line_pid_param.kp=50;
	motor_all.Cspeed = 400;   
	pid_mode_switch(is_Line);
//	scaner_set.EdgeIgnore=2;
	num = motor_all.Distance;
	while( motor_all.Distance-num <lenght)
	{
		scaner_set.EdgeIgnore=6;
		getline_error();
		vTaskDelay(2);
	}
	line_pid_param=origin_param;
	nodesr.nowNode.function=0;
	buzzer_off();
	nodesr.flag|=0x04;	//����·��
}


void South_Pole(float length)
{
	float num = 0;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1=gyroT_pid_param;
	float origin_turnM=motor_all.GyroT_speedMax;
	motor_all.GyroT_speedMax=500;
	motor_all.Cspeed = 600;
	pid_mode_switch(is_Line);
	line_pid_param.kp = 25;
	line_pid_param.kd = 15;
	gyroT_pid_param.kp=10;
	while(imu.pitch<Up_pitch)
	{
		vTaskDelay(2);
	}//����
	Rudder_control(170,0);//��վ����
	motor_all.Cspeed = 600;
	num = motor_all.Distance;
	while(motor_all.Distance - num < 90)
	{
		vTaskDelay(2);
	}		
	
	angle.AngleG = getAngleZ();
	pid_mode_switch(is_Gyro);
	motor_all.Gspeed = 600;

	while(Infrared_ahead == 0)
	{
		vTaskDelay(5);
	}  //ײ����
 
	num = motor_all.Distance;				
	while(motor_all.Distance - num < 20)//ǰ��һ�ξ���
	{
		vTaskDelay(5);
	}
	

	CarBrake();
	
	//myAction(); //�����˶���+Ѱ�����
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //������У��
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);

	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED1;
	uint8_t time=0;
	while (num-motor_all.Distance < 9)
	{
		if(time==0)
		{
			Rudder_control(150,2);//450���ַ���  150����
			vTaskDelay(100);
			Rudder_control(450,2);//450���ַ���  150����
			time=1;
		}
		vTaskDelay(2);
	}
	CarBrake();	
	Rudder_control(400,4);//130���ַ���  400����
	vTaskDelay(100);
	Rudder_control(130,4);//130���ַ���  400����
	vTaskDelay(50);
//��ת��
	bofang_zhiding(7);
	Turn_Angle_Relative(179);
	while(fabs(angle.AngleT - getAngleZ())>2)
	{
		vTaskDelay(2);
	}	
//	Turn_Angle_Relative(10);
//	while(fabs(angle.AngleT - getAngleZ())>2)
//	{
//		vTaskDelay(2);
//	}	
	motor_pid_clear();

	if(check_BW(nodesr.nowNode.nodenum))//���ֱ��صĶ���
	{
		Rudder_control(400,4);//130���ַ���  400����
        Rudder_control(150,2);//450���ַ���  150����
		bofang_zhiding(10);
		Turn_Angle360();
		Rudder_control(130,4);//130���ַ���  400����
        Rudder_control(450,2);//450���ַ���  150����
	}
//	while(Scaner.ledNum<=2)
//	{
//		vTaskDelay(2);
//	}
//	while(Scaner.ledNum>=10)
//	{
//		vTaskDelay(2);
//	}
//	CarBrake();
//	if((Scaner.detail&0XF800)!=0)
//	{
//		pid_mode_switch(is_No);
//		motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
//		int num = motor_all.Distance;
//		while(num - motor_all.Distance < 10)
//		{
//			vTaskDelay(2);
//		}
//		CarBrake();
//		Turn_Angle_Relative(15);
//		while (fabs(angle.AngleT-getAngleZ())>2)
//		{
//			vTaskDelay(2);
//		}
//	}
//	if((Scaner.detail&0X1F)!=0)
//	{
//		pid_mode_switch(is_No);
//		motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
//		int num = motor_all.Distance;
//		while(num - motor_all.Distance < 10)
//		{
//			vTaskDelay(2);
//		}
//		CarBrake();
//		Turn_Angle_Relative(-15);
//		while (fabs(angle.AngleT-getAngleZ())>2)
//		{
//			vTaskDelay(2);
//		}
//	}
	motor_pid_clear();
	motor_all.Cspeed = 600;
	pid_mode_switch(is_Line);
	Rudder_control(320,0);//������
	while(imu.pitch>Down_pitch)
	{
		vTaskDelay(2);
	}
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);
	}	
	line_pid_param = origin_param;  //�ָ�ԭ����PID����
	gyroT_pid_param=origin_param1;
	nodesr.nowNode.function=0;
	motor_all.GyroT_speedMax=origin_turnM;
	nodesr.flag|=0x04;	//����·��
}




void QQB_1(void)
{
	float num;
	struct PID_param origin_param = line_pid_param;
    struct PID_param origin_param1=gyroG_pid_param;
	gyroG_pid_param.kp=15;
	
//	gyroT_pid_param.outputMax = 200;
//	gyroT_pid_param.outputMin = 200;
	motor_pid_clear();
	pid_mode_switch(is_Line);
	motor_all.Cspeed = 700;  //�����ٽ���Ѱ��
	line_pid_param.kp = 45;    //����KP 
	nodesr.nowNode.flag|=LEFT_LINE;
	num=motor_all.Distance;
	while(motor_all.Distance-num<20)
	{
		vTaskDelay(2);
	}
	motor_all.Cspeed=400;
	if(nodesr.nowNode.nodenum==B9)
	{
	    scaner_set.CatchsensorNum = line_weight[4];    //�������Ȩֵ
		num=motor_all.Distance;
		while(imu.pitch<basic_p+5)
		{
			vTaskDelay(2);
		}
		pid_mode_switch(is_Turn);
		angle.AngleT=-95;
		while(fabs(getAngleZ()-angle.AngleT)>2)
		{
			vTaskDelay(2);
		}//ת��λ��
		angle.AngleG=getAngleZ();
		motor_all.Gspeed=250;
		pid_mode_switch(is_Gyro);

		num=motor_all.Distance;
		while(motor_all.Distance-num<55)
		{
			infrare_open=1;
			if(infrared.outside_left==1&infrared.outside_right==0)
			{
				angle.AngleG=getAngleZ()-2;
			}
			else if(infrared.outside_right==1&infrared.outside_left==0)
			{
					angle.AngleG=getAngleZ()+2;
			}
			else
			{
				angle.AngleG=getAngleZ();
			}
				vTaskDelay(2);
		}
		infrare_open=0;
			
		while(imu.pitch>basic_p-15)
		{
			 TIM1->CCR4 = 0; TIM1->CCR2 =1000;
			 TIM1->CCR3 = 0; TIM1->CCR1 =1000;
			 TIM2->CCR3 = 0; TIM2->CCR1 =1000;
			 TIM2->CCR4 = 0; TIM2->CCR2 =1000;
			vTaskDelay(2);
		}
		scaner_set.CatchsensorNum =0;
		pid_mode_switch(is_Line);
		
//		num=motor_all.Distance;
//		pid_mode_switch(is_Free);
//		while(motor_all.Distance-num<10)
//		{
//			 TIM1->CCR2 = 0; TIM1->CCR4 =1000;
//			 TIM1->CCR1 = 0; TIM1->CCR3 =1000;
//			 TIM2->CCR1 = 0; TIM2->CCR3 =3000;
//			 TIM2->CCR2 = 0; TIM2->CCR4 =3000;
//		}

//		if(Scaner.lineNum==0)
//		{
//			pid_mode_switch(is_Turn);
//			angle.AngleT=-45;
//			while(fabs(angle.AngleT-getAngleZ())>5)
//			{
//				if(Scaner.lineNum!=0)
//					break;
//				vTaskDelay(2);
//			}
//		}
//			CarBrake();
	}
	if(nodesr.nowNode.nodenum==B8)
	{
		scaner_set.CatchsensorNum = line_weight[4];    //�������Ȩֵ
		num=motor_all.Distance;
		while(imu.pitch<basic_p+5)
		{
			vTaskDelay(2);
		}
		pid_mode_switch(is_Turn);
		angle.AngleT=90;
		while(fabs(getAngleZ()-angle.AngleT)>2)
		{
			vTaskDelay(2);
		}//ת��λ��
		angle.AngleG=getAngleZ();
		motor_all.Gspeed=250;
		pid_mode_switch(is_Gyro);

		num=motor_all.Distance;
		while(motor_all.Distance-num<50)
		{
			infrare_open=1;
			buzzer_on();
			if(infrared.outside_left==1&infrared.outside_right==0)
			{
				angle.AngleG=getAngleZ()-2;
			}
			else if(infrared.outside_right==1&infrared.outside_left==0)
			{
					angle.AngleG=getAngleZ()+2;
			}
			else
			{
				angle.AngleG=getAngleZ();
			}
				vTaskDelay(2);
		}
		infrare_open=0;
		
//			CarBrake();
//			vTaskDelay(1000);
			angle.AngleG=getAngleZ();
			motor_all.Gspeed=200;
			pid_mode_switch(is_Gyro);
			while(imu.pitch>basic_p-5)
			{
				vTaskDelay(2);
			}
			while(Scaner.ledNum==0)
			{
				 TIM1->CCR2 = 0; TIM1->CCR4 =1000;
			     TIM1->CCR1 = 0; TIM1->CCR3 =1000;
			     TIM2->CCR1 = 0; TIM2->CCR3 =3000;
			     TIM2->CCR2 = 0; TIM2->CCR4 =3000;
			}
			if(Scaner.lineNum==0)
			{
				pid_mode_switch(is_Turn);
				angle.AngleT=45;
				while(fabs(angle.AngleT-getAngleZ())>5)
				{
					if(Scaner.lineNum!=0)
						break;
					vTaskDelay(2);
				}
			}
	//		CarBrake();
//		while(imu.pitch>basic_p-15)
//		{
//			 TIM1->CCR4 = 0; TIM1->CCR2 =1000;
//			 TIM1->CCR3 = 0; TIM1->CCR1 =1000;
//			 TIM2->CCR3 = 0; TIM2->CCR1 =1000;
//			 TIM2->CCR4 = 0; TIM2->CCR2 =1000;
//			vTaskDelay(2);
//		}
//		scaner_set.CatchsensorNum =0;
//		pid_mode_switch(is_Line);
		buzzer_off();
	}
	scaner_set.CatchsensorNum =0;
	pid_mode_switch(is_Line);
	motor_all.Cspeed=600;
	line_pid_param = origin_param;  //�ָ�ԭ����PID����
	gyroG_pid_param=origin_param1;
	nodesr.nowNode.function=0;
	scaner_set.CatchsensorNum =0;
	nodesr.flag|=0X04;	//����·��

}

void door()
{
	static uint8_t flag=0;
	pid_mode_switch(is_Line);
	motor_all.Cspeed=500;
	static uint8_t wait_cnt;
	while(1)
	{
		if(Scaner.ledNum>=8)
		{
			if(flag<=10)//ȥ
			{
				open_mv();
			}
			else
			{//����
				open_mvR();
			}
			CarBrake();
			//printf("%f,%f\r\n",motor_all.Lspeed,motor_all.Rspeed);
			vTaskDelay(500);//ʶ�����
			
			if(flag==11)//һ�ƻ���һ��һ�ƽ�
			{
				while(1)
				{
					if(color_R==1)
					{
						color_flag[3]=color_R;
						route_reset(6);
						bofang_zhiding(12);
						break;
					}
					else if(color_R==3)
					{
						color_flag[3]=color_R;
						bofang_zhiding(11);
						if(color_flag[0]==2)//��һ��·��Ϊ�ƣ����ĸ�Ϊ��
						{//һ��
							map.point -= 2;
							route[map.point] = N8;	
                            route[map.point+1]=N3;							
							Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
							while(fabs(angle.AngleT-getAngleZ())>3)
							{
								vTaskDelay(2);
							}
							nodesr.nowNode= Node[getNextConnectNode(N3,N10)];		//��������nowNode
							nodesr.nowNode.step = 25;
							nodesr.nowNode.flag =DRIGHT|DLEFT;
							nodesr.nowNode.speed=800;
							nodesr.flag=0x20;
							pid_mode_switch(is_Line);
							flag=12;
						}
						else if(color_flag[0]==3&color_flag[1]==2)//��һ��Ϊ�죬�ڶ���Ϊ�ƣ����ĸ�Ϊ�죬��������Ϊ��
						{//һ��һ��
							Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
							while(fabs(angle.AngleT-getAngleZ())>3)
							{
								vTaskDelay(2);
							}
							pid_mode_switch(is_Line);
							route_reset(7);
							flag=0;//��ֹ�´��ٽ�
						}
						break;
					}
					else{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if(wait_cnt==50)
					{
						wait_cnt=0;
						open_mvR();
					}
				}
				close_mvR();
				pid_mode_switch(is_Line);
				motor_all.Cspeed=nodesr.nowNode.speed;
				nodesr.nowNode.function=1;
				return;
			}
			if(flag==12)//���ơ������������졿��
			{
				while(1)
				{
					if(color_R==1)
					{
						color_flag[2]=color_R;
						route_reset(8);
						bofang_zhiding(12);
						motor_pid_clear();
//						CarBrake();
//						vTaskDelay(5000);
						break;
					}
					else if(color_R==3)
					{
						color_flag[2]=color_R;
						bofang_zhiding(11);
						Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
						while(fabs(angle.AngleT-getAngleZ())>3)
						{
							vTaskDelay(2);
						}//��ת��
						pid_mode_switch(is_Line);
						motor_all.Cspeed=nodesr.nowNode.speed;
						nodesr.nowNode= Node[getNextConnectNode(N3,N8)];		//��������nowNode
						nodesr.nowNode.step = 40;
						nodesr.nowNode.flag =DRIGHT|DLEFT|SLOWDOWN;
						nodesr.nowNode.speed=650;
						route_reset(11);
						bofang_zhiding(13);
						close_mvR();
						break;
					}
					else{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if(wait_cnt==50)
					{
						wait_cnt=0;
						open_mvR();
					}
				}
				flag=0;
				close_mvR();
				pid_mode_switch(is_Line);
				motor_all.Cspeed=nodesr.nowNode.speed;
				nodesr.nowNode.function=1;
				return;
			}
			if(flag==2)//N3-N8���ж�
			{
				flag=0;
				while(1)
				{
					if(color==1)
					{
						color_flag[2]=color;
						route_reset(4);
						bofang_zhiding(12);
						close_mv();
						break;
					}
					else if(color==2)
					{	
						color_flag[2]=color;
						route_reset(9);
						bofang_zhiding(13);
						close_mv();
						break;
					}
					else{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if(wait_cnt==50)
					{
						wait_cnt=0;
						open_mv();
					}
				}
				pid_mode_switch(is_Line);
				motor_all.Cspeed=nodesr.nowNode.speed;
				nodesr.nowNode.function=1;
				return;
			}
			if(flag==1)//��һ���Ǻ�ƽ����ڶ����ж�
			{
				while(1)
				{
					if(color==3)
					{   
						color_flag[1]=color;
						bofang_zhiding(11);
						Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
						while(fabs(angle.AngleT-getAngleZ())>3)
						{
							vTaskDelay(2);
						}//��ת��
						pid_mode_switch(is_Line);
						motor_all.Cspeed=nodesr.nowNode.speed;
						flag=2;
						route_reset(1);
						close_mv();
						return;		
					}
					else if(color==1|color==2)
					{
						if(color==1)
						{
							color_flag[1]=color;
							bofang_zhiding(12);
							pid_mode_switch(is_Line);
						    motor_all.Cspeed=nodesr.nowNode.speed;
						    route_reset(2);
						    flag=0;
						}
						else if(color==2)
						{
							color_flag[1]=color;
							bofang_zhiding(13);
							pid_mode_switch(is_Line);
						    motor_all.Cspeed=nodesr.nowNode.speed;
							route_reset(10);
							flag=11;
						}
						close_mv();
						return;
					}
					else{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if(wait_cnt==50)
					{
						wait_cnt=0;
						open_mv();
					}
				}
				
			}
			if(flag==0)//��һ��
			{
				while(1)
				{
					if(color==3)
					{
						color_flag[0]=color;
						bofang_zhiding(11);
						map.point -= 2;
						route[map.point] = N8;				
						Turn_Angle_Relative(181);	//	ת����ǰ��㷽��
						while(fabs(angle.AngleT-getAngleZ())>3)
						{
							vTaskDelay(2);
						}
						nodesr.nowNode= Node[getNextConnectNode(N12,N5)];		//��������nowNode
						nodesr.nowNode.step = 25;
						nodesr.nowNode.flag =DRIGHT|DLEFT;
						nodesr.nowNode.speed=800;
						pid_mode_switch(is_Line);
						nodesr.flag|=0x20;
						flag=1;
						close_mv();
						return;				
					}
					else if(color==1)
					{
						color_flag[0]=color;
						bofang_zhiding(12);
						pid_mode_switch(is_Line);
						motor_all.Cspeed=nodesr.nowNode.speed;
						route_reset(3);
						flag=0;
						close_mv();
						return;
					}
					else if(color==2)
					{
						color_flag[0]=color; 
						bofang_zhiding(13);
						pid_mode_switch(is_Line);
						motor_all.Cspeed=nodesr.nowNode.speed;
						route_reset(5);
						flag=11;//����ʱ��
						close_mv();
						return;
					}
					else{
						CarBrake();
					}
					vTaskDelay(2);
					wait_cnt++;
					if(wait_cnt==50)
					{
						wait_cnt=0;
						open_mv();
					}
				}
				
			}
		}
		
	}
	
}


void route_reset(u8 flag)//ƽ̨������
{
	static u8 temp=0,i=0;
	temp = map.point;
	i=0;
	if(flag==1)//�����ƽ̨
	{	
		while(1)
		{	
			route[temp++]=door1route[i++];		//·������
			if(door1route[i]==255)
			{
				route[temp]=door1route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N8,N5)];		//��������nowNode
				nodesr.nowNode.step=80;
				nodesr.nowNode.function=1;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==2)//һ��  ��һ�ƻ��� 
	{
		while(1)
		{	
			route[temp++]=door2route[i++];		//·������
			if(door2route[i]==255)
			{
				route[temp]=door2route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N8)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==3)
	{
		while(1)
		{	
			route[temp++]=door3route[i++];		//·������
			if(door3route[i]==255)
			{
				route[temp]=door3route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N12)];		//��������nowNode
				nodesr.nowNode.step=50;
				nodesr.nowNode.speed=800;
				nodesr.nowNode.function=1;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==4)
	{
		while(1)
		{	
			route[temp++]=door4route[i++];		//·������
			if(door4route[i]==255)
			{
				route[temp]=door4route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=20;
				Node[getNextConnectNode(N8,N3)].function=1;
				Node[getNextConnectNode(N8,N3)].step=200;
				Node[getNextConnectNode(N8,N3)].speed=1400;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==5)
	{
		while(1)
		{	
			route[temp++]=door5route[i++];		//·������
			if(door5route[i]==255)
			{
				route[temp]=door5route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N12)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=20;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==6)
	{

		while(1)
		{	
			route[temp++]=door6route[i++];		//·������
			if(door6route[i]==255)
			{
				route[temp]=door6route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N10,N3)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=20;
				nodesr.flag|=0x80;

				break;
			}
		}
	}
	else if(flag==7)
	{
		while(1)
		{	
			route[temp++]=door7route[i++];		//·������
			if(door7route[i]==255)
			{
				route[temp]=door7route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N10)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=50;
				Node[getNextConnectNode(N8,N3)].step=170;
				Node[getNextConnectNode(N8,N3)].function=1;			
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==8)
	{
		while(1)
		{	
			route[temp++]=door6route[i++];		//·������
			if(door6route[i]==255)
			{
				route[temp]=door6route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N8,N3)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=50;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==9)
	{
		while(1)
		{	
			route[temp++]=door9route[i++];		//·������
			if(door9route[i]==255)
			{
				route[temp]=door9route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=50;
				Node[getNextConnectNode(N10,N3)].function=1;
				Node[getNextConnectNode(N10,N3)].step=250;
				Node[getNextConnectNode(N10,N3)].speed=1400;
//				Node[getNextConnectNode(N10,N3)].flag=DRIGHT|DLEFT|SLOWDOWN;
//				Node[getNextConnectNode(N10,N3)].angle=-90;
//				Node[getNextConnectNode(N10,N3)].nodenum=N3;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==10)
	{
		while(1)
		{	
			route[temp++]=door10route[i++];		//·������
			if(door10route[i]==255)
			{
				route[temp]=door10route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N8)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=50;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==11)
	{
		while(1)
		{	
			route[temp++]=door11route[i++];		//·������
			if(door11route[i]==255)
			{
				route[temp]=door11route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//��������nowNode
				nodesr.nowNode.function=1;
				nodesr.nowNode.step=50;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
}
void undermou(void)
{
	float num = 0;
	while(imu.pitch > -3);
	motor_all.Cspeed = 60;
	num = motor_all.Distance;
	while(motor_all.Distance - num < 50);
	buzzer_on();
	if(nodesr.nowNode.nodenum == N14)
	{
		motor_all.Cspeed = 110;
		num = motor_all.Distance;
		while(motor_all.Distance - num < 100);
		motor_all.Cspeed = 60;
	}
	while(!deal_arrive());
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}
void S_curve(void)
{	
	float num;
	struct PID_param origin_param = line_pid_param;
	
	num = motor_all.Distance;
	motor_all.Cspeed = nodesr.nowNode.speed;
	
	if (nodesr.nextNode.nodenum == N13)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance-num) < 110);
		motor_all.Cspeed = 60;
		scaner_set.CatchsensorNum = line_weight[11];  //C1->C2
		line_pid_param.kp = 40;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10);
	}
	else if (nodesr.nextNode.nodenum == C1)
	{
		motor_all.Cspeed = 110;
		while (fabsf(motor_all.Distance-num) < 60);
		scaner_set.CatchsensorNum = line_weight[5];
		line_pid_param.kp = 70;
		while (fabsf(getAngleZ() - nodesr.nextNode.angle) > 10);
	}
	
	line_pid_param = origin_param;
	scaner_set.CatchsensorNum = 0;
	
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}


void ignore_node(void)
{
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//����·��
}
void get_newroute(void)
{
	mapInit1();
	map.point=0;
	for(int i=0;i<126;i++)
	{
		if(Node[i].function==DOOR)
		{
			Node[i].step*=2;
			Node[i].function=1;
		}
	}
	if(color_flag[0]==1)//��һ���ſ�
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,S2,N6,P4,N5,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==2&color_flag[3]==1)
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==2&color_flag[3]==3&color_flag[2]==1)
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	} 
	if(color_flag[0]==2&color_flag[3]==3&color_flag[2]==3&color_flag[1]==1)
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==3&color_flag[1]==1)
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,N8,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==3&color_flag[1]==2&color_flag[3]==1)
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,N8,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==3&color_flag[1]==2&color_flag[3]==3&color_flag[2]==1)
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,N8,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,P3,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==3&color_flag[1]==3&color_flag[3]==2&color_flag[2]==1)//���������ȥ��
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,P3,N3,N8,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==3&color_flag[1]==3&color_flag[2]==2)//���������ȥ��
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,P3,N3,N8,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
}
void zhunbei(void)
{
		Rudder_control(170,0);//��վ����
	Rudder_control(250,1);//�м�250 180�ұ� 320��� 
//ƽ̨���³�����
	
	while(Infrared_ahead==0)
	{
		vTaskDelay(2);
	}//��ѭ���е���
	while(Infrared_ahead==1)
	{
		vTaskDelay(2);
	}//��ѭ���޵���
	bofang_zhiding(14);
	Rudder_control(320,1);//��ת
	vTaskDelay(100);
	Rudder_control(150,1);//��ת
	vTaskDelay(100);
	Rudder_control(320,1);//��ת
	vTaskDelay(100);
	Rudder_control(150,1);//��ת
	vTaskDelay(100);
	Rudder_control(250,1);//�м�
	Rudder_control(320,0);//������
	{
		runWithAngle(nodesr.nowNode.angle,500);
			while(imu.pitch>Down_pitch)
			{
				vTaskDelay(2);
			}//����
			while(imu.pitch<After_down)
			{
				vTaskDelay(2);
			}//�������
				//·�̼�¼����
			encoder_clear();
			pid_mode_switch(is_Line); 
	        motor_all.Cspeed = 500;
      }
}


