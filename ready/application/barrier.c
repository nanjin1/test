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
uint8_t  value;//openmv接口
uint8_t special_arrive=0;
//防抖动系列函数
//1._shake在平台识别过程中，防止因一些意外因素导致车提前停止,同时完成撞板的目的
//2._Rshake用于判断其是否经过波浪板
//3._pshake识别处于下坡状态
void Anti_shake(int Uneed_time)
{
	uint32_t i = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
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
//	uint32_t i = 0, cnt = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
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
//	uint32_t i = 0;  //循环变量设置成unit32_t避免for循环最后的奇怪指令
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

void Stage()		//flag==1时取绝度角度，flag==0时取相对角度
{
	float num = 0; //distan = 0;
	nodesr.nowNode.flag=0;
//	pid_mode_switch(is_Line);
//    motor_all.Cspeed = 600;
//	struct PID_param origin_param = gyroT_pid_param;
	gyroT_pid_param.kp=40;
	float origin_turnM=motor_all.GyroT_speedMax;
	motor_all.GyroT_speedMax=550;
    Rudder_control(170,0);//人站起来
//	if ((Scaner.detail & 0X0180) == 0X0180) //如果在最中间位置
//	{
//		angle.AngleG = getAngleZ();//自平衡走的角度
//		pid_mode_switch(is_Gyro);
//		motor_all.Gspeed = 400;
//	}
//	else{
//		angle.AngleG = nodesr.nowNode.angle;//自平衡走的角度
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
//	}//开始上平台
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
	}//上完桥
   
	while(Infrared_ahead == 0)
	{
		vTaskDelay(5);
	}  //撞挡板
	
	
//	num = motor_all.Distance;

//	while(motor_all.Distance - num < 20)//前进一段距离
//	{
//		vTaskDelay(5);
//	}

	//delay_ms(300);
	CarBrake();
	
	//HAL_Delay(1000);
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	num = motor_all.Distance;				//后退一段距离
	
	pid_mode_switch(is_No);
//	motor_L0.target = motor_L1.target = BACK_SPEED;
//	motor_R0.target = motor_R1.target = BACK_SPEED;
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED;
	uint8_t time=0;
	while (num-motor_all.Distance < 4) //9
	{
		if(time==0)
		{
			Rudder_control(150,2);//450左手放下  150举起
			vTaskDelay(100);
			Rudder_control(450,2);//450左手放下  150举起
			time=1;
		}
		vTaskDelay(2);
	}
	CarBrake();	
	Rudder_control(400,4);//130右手放下  400举起
	vTaskDelay(100);
	Rudder_control(130,4);//130右手放下  400举起
	vTaskDelay(50);

//	if(nodesr.nowNode.nodenum!=P7&nodesr.nowNode.nodenum!=P8)
//	{
//		while(BW_add==0)
//		{
//			vTaskDelay(2);
//		}//等待宝藏
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
	{//平台4
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
	if(check_BW(nodesr.nowNode.nodenum))//发现宝藏的动作
	{
		Rudder_control(400,4);//130右手放下  400举起
        Rudder_control(150,2);//450左手放下  150举起
		bofang_zhiding(10);
		Turn_Angle360();
		Rudder_control(130,4);//130右手放下  400举起
        Rudder_control(450,2);//450左手放下  150举起
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
    Rudder_control(320,0);//人躺下
	pid_mode_switch(is_Line);
	motor_all.Cspeed=800;
	nodesr.nowNode.function=0;	//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}
//特殊结点
void Special_Node()
{
	
	if(((nodesr.nowNode.flag&DRIGHT)==DRIGHT)&((nodesr.nowNode.flag&CRIGHT)==CRIGHT)&(nodesr.nowNode.nodenum==N5))//N4-N5右循迹 左循迹 边缘忽略
	{//N4-N5
		nodesr.nowNode.flag&=(~RIGHT_LINE);//取消右循迹标志位
		nodesr.nowNode.flag|=LEFT_LINE;//左循迹
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}//右分岔
		nodesr.nowNode.flag&=(~CRIGHT);//取消右分岔标志位
	    while(deal_arrive()!=1)//右半边天
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
		nodesr.nowNode.flag|=LEFT_LINE;//左循迹
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}
		num=motor_all.Distance;
		while(motor_all.Distance-num<10)//第一个左分岔路口再走10厘米
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
		nodesr.nowNode.flag|=RIGHT_LINE;//右循迹
		while(deal_arrive()!=1)
		{
			vTaskDelay(2);
		}
		num=motor_all.Distance;
		while(motor_all.Distance-num<10)//第一个左分岔路口再走10厘米
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
//	if(((nodesr.nowNode.flag&CRIGHT)==CRIGHT)&((nodesr.nowNode.flag&CLEFT)==CLEFT))//N5-N6  P4-N6先左循迹后右循迹 
//	{
//		angle.AngleT=getAngleZ();
//		pid_mode_switch(is_Gyro);
////		nodesr.nowNode.flag|=LEFT_LINE;//左循迹
////		while(deal_arrive()!=1)
////		{
////			vTaskDelay(2);
////		}//检测到右分岔
////		nodesr.nowNode.flag&=(~CRIGHT);//取消右分岔标志位
////		while(deal_arrive()!=1)
////		{
////			vTaskDelay(2);
////		}//检测到左分岔
////		nodesr.nowNode.flag&=(~LEFT_LINE);//取消左循迹
////		nodesr.nowNode.flag|=RIGHT_LINE;//右循迹
////		special_arrive=1;
//	}
}

//平台二的动作
void Stage_P2()		//flag==1时取绝度角度，flag==0时取相对角度
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
	Turn_Angle_Relative(181);		//转>=180度，
	while (fabs(angle.AngleT-getAngleZ())>2)
	{
		vTaskDelay(5);
	}
	nodesr.nowNode.function=0;	//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}

/***************************************************
过长桥
************************************************/

void Barrier_Bridge(float step,float speed)	//过长桥
{
	float num=0;
	buzzer_on();
	num=motor_all.Distance;
	motor_all.Gspeed = 800;    //自平衡速度
	float now_angle=0;

	angle.AngleG = getAngleZ();//自平衡走的角度
	pid_mode_switch(is_Gyro);
	now_angle=getAngleZ();

	struct PID_param origin_param = gyroG_pid_param;
	gyroG_pid_param.kp = 1.8;//原来3.7 4.8 7.8 外面可能7.8 6.8卡卡的	
	gyroG_pid_param.ki=0;
	while(imu.pitch <= Up_pitch)	//还在平地,出循环就是上桥中
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
//			angle.AngleG = getAngleZ();//正确的角度
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
//	}//下桥中
	angle.AngleG=now_angle;
//	pid_mode_switch(is_Free);
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);	
//		TIM1->CCR4 = 0; TIM1->CCR2 = 5000;
//			TIM1->CCR1 = 0; TIM1->CCR3 = 5000;
//			TIM2->CCR4 = 0; TIM2->CCR2 = 5000;
//			TIM2->CCR1 = 0; TIM2->CCR3 = 5000;
	}//出循环下完在平地
	pid_mode_switch(is_Line);
	buzzer_off();
		//路程记录清零
	motor_all.Distance = 0;
	motor_all.encoder_avg = 0;
	nodesr.nowNode.function = 0;
	nodesr.flag |= 0X04;  //到达路口
	gyroG_pid_param=origin_param;
}

//过楼梯
void Barrier_Hill(uint8_t order)  //楼梯数量
{
	float num=0;
	angle.AngleG = getAngleZ();
	buzzer_on();
	struct PID_param origin_param = line_pid_param;
	line_pid_param.kp=45;
	motor_all.Cspeed = 250;   //原来是400
	pid_mode_switch(is_Line);
	while(imu.pitch<basic_p+10)
	{
		vTaskDelay(2);
	}
    motor_all.Cspeed = 400; 	
//	while ( imu.pitch >10+basic_p )	
//	{
//		vTaskDelay(2);
//	}//开始上楼梯
	while(imu.pitch>-10+basic_p)
	{
		vTaskDelay(2);
	}//开始下楼梯
	num=motor_all.Distance;
    while(imu.pitch<basic_p-6)
	{
		vTaskDelay(2);
		if(motor_all.Distance-num>20)
			break;
	}//下完楼梯
    buzzer_off();
	line_pid_param = origin_param;
	nodesr.nowNode.function=0;//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}


void Sword_Mountain()
{
float num;
	struct PID_param origin_param = line_pid_param;
	struct PID_param origin_param1 = gyroG_pid_param;
	num = motor_all.Distance;
	motor_all.Cspeed = 600; 
	line_pid_param.kp = 50;
	
	
	//mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	while(motor_all.Distance - num < 25)//强矫正循迹位置
	{
		vTaskDelay(2);
	}
	angle.AngleG = getAngleZ();
	
	gyroG_pid_param.kp = 2.4;   //拉大刀山的自平衡kp
	motor_all.Gspeed = 600;
	pid_mode_switch(is_Gyro);
    num=motor_all.Distance;
	while(imu.pitch < After_up)//出循环上刀山
	{
		vTaskDelay(2);
		if(motor_all.Distance-num>25)
			break;
	}
	buzzer_on();
	

	while(imu.pitch > After_down)//出循环下刀山
	{
		vTaskDelay(2);
	}
	gyroG_pid_param = origin_param1;
	line_pid_param = origin_param;
	buzzer_off();
	
	 
	nodesr.nowNode.function=0;//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}

/*****************************************************************************
上珠峰，包含下珠峰
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
	}//刚上桥
	motor_all.Cspeed = speed; //最开始还没上坡的速度 90
	motor_all.Cincrement = 20;
	pid_mode_switch(is_Line);
    num=motor_all.Distance;
	motor_all.Cincrement = 20; //上坡的速度  2
	while(motor_all.Distance-num<70)//上坡走循迹
	{
		vTaskDelay(2);
	}
	motor_all.Gspeed = speed; //第一个平地自平衡	
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
	}//第二个坡
	buzzer_off();
	
	Rudder_control(170,0);//320躺下 170站起来
	motor_all.Cspeed = speed; 
	motor_all.Cincrement = 20;
	pid_mode_switch(is_Line);
	num=motor_all.Distance;
	while(motor_all.Distance-num<80)//上坡走循迹
	{
		vTaskDelay(2);
	}
	motor_all.Gspeed = speed; //第二次自平衡	
	angle.AngleG=getAngleZ();
    pid_mode_switch(is_Gyro);
	buzzer_on();
    while(imu.pitch>After_up)
	{
		vTaskDelay(2);
	}//第二次上平台
	buzzer_off();
	while(Infrared_ahead == 0)
	{
		vTaskDelay(5);
	}  //撞挡板
	num = motor_all.Distance;				
	while(motor_all.Distance - num < 20)//前进一段距离
	{
		vTaskDelay(5);
	}
	CarBrake();
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
   	
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);
	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED1;
	uint8_t time=0;
	while (num-motor_all.Distance < 9)
	{
		if(time==0)
		{
			Rudder_control(150,2);//450左手放下  150举起
			vTaskDelay(100);
			Rudder_control(450,2);//450左手放下  150举起
			time=1;
		}
		vTaskDelay(2);
	}
	CarBrake();	
	Rudder_control(400,4);//130右手放下  400举起
	vTaskDelay(100);
	Rudder_control(130,4);//130右手放下  400举起
	vTaskDelay(50);
	motor_all.GyroT_speedMax=500;
	bofang_zhiding(8);
	Turn_Angle_Relative(179);//转180
	while(fabs(angle.AngleT - getAngleZ())>2) //判断误差
	{
		vTaskDelay(2);
	}
	 motor_pid_clear();
//	CarBrake();
//   vTaskDelay(300);
	if(check_BW(nodesr.nowNode.nodenum))//发现宝藏的动作
	{
		motor_pid_clear();
		Rudder_control(400,4);//130右手放下  400举起
        Rudder_control(150,2);//450左手放下  150举起
		bofang_zhiding(10);
		Turn_Angle360();
		Rudder_control(130,4);//130右手放下  400举起
        Rudder_control(450,2);//450左手放下  150举起
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
	nodesr.nowNode.function=0;//清除障碍标志
	nodesr.flag|=0x04;	//到达路口
}

/*****************************************************************************
函数功能：下珠峰，在转完180后调用
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
	{//保护机制
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
	pid_mode_switch(is_Line);//照到白线开循迹
	motor_all.Cspeed=450;
//	pid_mode_switch(is_Gyro);
//	angle.AngleG=getAngleZ();
//	motor_all.Gspeed=450;
	while(imu.pitch>Down_pitch)
	{
		vTaskDelay(2);
	}//下坡
	
	num=motor_all.Distance;
	while(motor_all.Distance-num<40)
	{
		vTaskDelay(2);
	}//先慢速
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
	}//第一个平地
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
	}//下坡
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);
	}//在地面
	buzzer_off();
}


void view()//打景点	
{	
	float num = 0;
	while(Infrared_ahead == 0)		//撞挡板
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
	nodesr.flag|=0x04;	//到达路口
}

void view1()//打景点	
{	
	pid_mode_switch(is_Line);
	while(Infrared_ahead == 0);		//撞挡板
	delay_ms(100);
	//mpuZreset(gyro.yaw,nodesr.nowNode.angle-10);
	CarBrake();
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}

void back()
{
	pid_mode_switch(is_No);
	
	motor_L0.target = motor_L1.target = BACK_SPEED;
	motor_R0.target = motor_R1.target = BACK_SPEED;
	
	while(infrared.outside_left == 0 && infrared.outside_right == 0);
	CarBrake();
	//转绝对角度
	angle.AngleT = nodesr.nextNode.angle;
	pid_mode_switch(is_Turn);
	while(fabs(angle.AngleT - getAngleZ())>5);
	
	motor_pid_clear();
	pid_mode_switch(is_Line);
	//motor_all.Cspeed=15;
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
}
/*****************************************************************************
过波浪板
速度   ，  波浪板长度（cm）
长度主要是防止误判提前进入波浪板，然后提前放下造成严重失误。
先走这一段长度，保证能进入波浪板，且避免提前误判成已经走去波浪板。
*****************************************************************************/
void Barrier_WavedPlate(float lenght)//波浪板长度
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
	nodesr.flag|=0x04;	//到达路口
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
	}//上坡
	Rudder_control(170,0);//人站起来
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
	}  //撞挡板
 
	num = motor_all.Distance;				
	while(motor_all.Distance - num < 20)//前进一段距离
	{
		vTaskDelay(5);
	}
	

	CarBrake();
	
	//myAction(); //机器人动作+寻宝检测
	
	mpuZreset(imu.yaw, nodesr.nowNode.angle);  //陀螺仪校正
	
	num = motor_all.Distance;
	pid_mode_switch(is_No);

	motor_all.Lspeed = motor_all.Rspeed = BACK_SPEED1;
	uint8_t time=0;
	while (num-motor_all.Distance < 9)
	{
		if(time==0)
		{
			Rudder_control(150,2);//450左手放下  150举起
			vTaskDelay(100);
			Rudder_control(450,2);//450左手放下  150举起
			time=1;
		}
		vTaskDelay(2);
	}
	CarBrake();	
	Rudder_control(400,4);//130右手放下  400举起
	vTaskDelay(100);
	Rudder_control(130,4);//130右手放下  400举起
	vTaskDelay(50);
//多转了
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

	if(check_BW(nodesr.nowNode.nodenum))//发现宝藏的动作
	{
		Rudder_control(400,4);//130右手放下  400举起
        Rudder_control(150,2);//450左手放下  150举起
		bofang_zhiding(10);
		Turn_Angle360();
		Rudder_control(130,4);//130右手放下  400举起
        Rudder_control(450,2);//450左手放下  150举起
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
	Rudder_control(320,0);//人躺下
	while(imu.pitch>Down_pitch)
	{
		vTaskDelay(2);
	}
	while(imu.pitch<After_down)
	{
		vTaskDelay(2);
	}	
	line_pid_param = origin_param;  //恢复原来的PID参数
	gyroT_pid_param=origin_param1;
	nodesr.nowNode.function=0;
	motor_all.GyroT_speedMax=origin_turnM;
	nodesr.flag|=0x04;	//到达路口
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
	motor_all.Cspeed = 700;  //给低速进入寻线
	line_pid_param.kp = 45;    //拉大KP 
	nodesr.nowNode.flag|=LEFT_LINE;
	num=motor_all.Distance;
	while(motor_all.Distance-num<20)
	{
		vTaskDelay(2);
	}
	motor_all.Cspeed=400;
	if(nodesr.nowNode.nodenum==B9)
	{
	    scaner_set.CatchsensorNum = line_weight[4];    //给予左边权值
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
		}//转正位置
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
		scaner_set.CatchsensorNum = line_weight[4];    //给予左边权值
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
		}//转正位置
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
	line_pid_param = origin_param;  //恢复原来的PID参数
	gyroG_pid_param=origin_param1;
	nodesr.nowNode.function=0;
	scaner_set.CatchsensorNum =0;
	nodesr.flag|=0X04;	//到达路口

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
			if(flag<=10)//去
			{
				open_mv();
			}
			else
			{//返回
				open_mvR();
			}
			CarBrake();
			//printf("%f,%f\r\n",motor_all.Lspeed,motor_all.Rspeed);
			vTaskDelay(500);//识别红绿
			
			if(flag==11)//一黄或者一红一黄进
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
						if(color_flag[0]==2)//第一个路口为黄，第四个为红
						{//一黄
							map.point -= 2;
							route[map.point] = N8;	
                            route[map.point+1]=N3;							
							Turn_Angle_Relative(181);	//	转到当前结点方向
							while(fabs(angle.AngleT-getAngleZ())>3)
							{
								vTaskDelay(2);
							}
							nodesr.nowNode= Node[getNextConnectNode(N3,N10)];		//重新设置nowNode
							nodesr.nowNode.step = 25;
							nodesr.nowNode.flag =DRIGHT|DLEFT;
							nodesr.nowNode.speed=800;
							nodesr.flag=0x20;
							pid_mode_switch(is_Line);
							flag=12;
						}
						else if(color_flag[0]==3&color_flag[1]==2)//第一个为红，第二个为黄，第四个为红，第三个必为绿
						{//一红一黄
							Turn_Angle_Relative(181);	//	转到当前结点方向
							while(fabs(angle.AngleT-getAngleZ())>3)
							{
								vTaskDelay(2);
							}
							pid_mode_switch(is_Line);
							route_reset(7);
							flag=0;//防止下次再进
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
			if(flag==12)//【黄】【】【】【红】进
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
						Turn_Angle_Relative(181);	//	转到当前结点方向
						while(fabs(angle.AngleT-getAngleZ())>3)
						{
							vTaskDelay(2);
						}//先转弯
						pid_mode_switch(is_Line);
						motor_all.Cspeed=nodesr.nowNode.speed;
						nodesr.nowNode= Node[getNextConnectNode(N3,N8)];		//重新设置nowNode
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
			if(flag==2)//N3-N8灯判断
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
			if(flag==1)//第一次是红灯进，第二次判断
			{
				while(1)
				{
					if(color==3)
					{   
						color_flag[1]=color;
						bofang_zhiding(11);
						Turn_Angle_Relative(181);	//	转到当前结点方向
						while(fabs(angle.AngleT-getAngleZ())>3)
						{
							vTaskDelay(2);
						}//先转弯
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
			if(flag==0)//第一次
			{
				while(1)
				{
					if(color==3)
					{
						color_flag[0]=color;
						bofang_zhiding(11);
						map.point -= 2;
						route[map.point] = N8;				
						Turn_Angle_Relative(181);	//	转到当前结点方向
						while(fabs(angle.AngleT-getAngleZ())>3)
						{
							vTaskDelay(2);
						}
						nodesr.nowNode= Node[getNextConnectNode(N12,N5)];		//重新设置nowNode
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
						flag=11;//返回时进
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


void route_reset(u8 flag)//平台三下来
{
	static u8 temp=0,i=0;
	temp = map.point;
	i=0;
	if(flag==1)//俩红冲平台
	{	
		while(1)
		{	
			route[temp++]=door1route[i++];		//路线连接
			if(door1route[i]==255)
			{
				route[temp]=door1route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N8,N5)];		//重新设置nowNode
				nodesr.nowNode.step=80;
				nodesr.nowNode.function=1;
				nodesr.flag|=0x80;
				break;
			}
		}
	}
	else if(flag==2)//一红  另一黄或绿 
	{
		while(1)
		{	
			route[temp++]=door2route[i++];		//路线连接
			if(door2route[i]==255)
			{
				route[temp]=door2route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N8)];		//重新设置nowNode
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
			route[temp++]=door3route[i++];		//路线连接
			if(door3route[i]==255)
			{
				route[temp]=door3route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N12)];		//重新设置nowNode
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
			route[temp++]=door4route[i++];		//路线连接
			if(door4route[i]==255)
			{
				route[temp]=door4route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//重新设置nowNode
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
			route[temp++]=door5route[i++];		//路线连接
			if(door5route[i]==255)
			{
				route[temp]=door5route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N12)];		//重新设置nowNode
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
			route[temp++]=door6route[i++];		//路线连接
			if(door6route[i]==255)
			{
				route[temp]=door6route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N10,N3)];		//重新设置nowNode
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
			route[temp++]=door7route[i++];		//路线连接
			if(door7route[i]==255)
			{
				route[temp]=door7route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N10)];		//重新设置nowNode
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
			route[temp++]=door6route[i++];		//路线连接
			if(door6route[i]==255)
			{
				route[temp]=door6route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N8,N3)];		//重新设置nowNode
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
			route[temp++]=door9route[i++];		//路线连接
			if(door9route[i]==255)
			{
				route[temp]=door9route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//重新设置nowNode
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
			route[temp++]=door10route[i++];		//路线连接
			if(door10route[i]==255)
			{
				route[temp]=door10route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N5,N8)];		//重新设置nowNode
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
			route[temp++]=door11route[i++];		//路线连接
			if(door11route[i]==255)
			{
				route[temp]=door11route[i];	
				nodesr.nowNode	= Node[getNextConnectNode(N3,N8)];		//重新设置nowNode
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
	nodesr.flag|=0x04;	//到达路口
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
	nodesr.flag|=0x04;	//到达路口
}


void ignore_node(void)
{
	nodesr.nowNode.function=0;
	nodesr.flag|=0x04;	//到达路口
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
	if(color_flag[0]==1)//第一个门开
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
	if(color_flag[0]==3&color_flag[1]==3&color_flag[3]==2&color_flag[2]==1)//从最外面出去吧
	{
		u8 temp[100]={N1,P1,N1,B2,N4,N6,P4,N5,P3,N3,N8,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,B3,N2,P2,0XFF};
		for(int i=0;i<100;i++)
		{
			route[i]=temp[i];
			if(temp[i]==0xff)
				break;
		}
	}
	if(color_flag[0]==3&color_flag[1]==3&color_flag[2]==2)//从最外面出去吧
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
		Rudder_control(170,0);//人站起来
	Rudder_control(250,1);//中间250 180右边 320左边 
//平台的下车代码
	
	while(Infrared_ahead==0)
	{
		vTaskDelay(2);
	}//出循环有挡板
	while(Infrared_ahead==1)
	{
		vTaskDelay(2);
	}//出循环无挡板
	bofang_zhiding(14);
	Rudder_control(320,1);//左转
	vTaskDelay(100);
	Rudder_control(150,1);//右转
	vTaskDelay(100);
	Rudder_control(320,1);//左转
	vTaskDelay(100);
	Rudder_control(150,1);//右转
	vTaskDelay(100);
	Rudder_control(250,1);//中间
	Rudder_control(320,0);//人躺下
	{
		runWithAngle(nodesr.nowNode.angle,500);
			while(imu.pitch>Down_pitch)
			{
				vTaskDelay(2);
			}//下桥
			while(imu.pitch<After_down)
			{
				vTaskDelay(2);
			}//下桥完毕
				//路程记录清零
			encoder_clear();
			pid_mode_switch(is_Line); 
	        motor_all.Cspeed = 500;
      }
}


