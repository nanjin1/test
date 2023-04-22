#include "map.h"
#include "barrier.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "scaner.h"
#include "imu_task.h"
#include "turn.h"
#include "speed_ctrl.h"
#include "motor_task.h"
#include "bsp_linefollower.h"
#include "math.h"
#include "bsp_buzzer.h"
#include "motor_task.h"
#include "encoder.h"
#include "uart.h"
#include "openmv.h"
struct Map_State map = {0,0};
//u8 route[100] = {C8, C4, N20, P7, N20, 0XFF};

//u8 route[100] = {C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,0XFF};

//u8 route[100] = {N9,B9,N7,P5,N7,0XFF};

//u8 route [100] = {B1,N1,P1,N1,B2,N4,N5,N6,P4,N6,N5,N4,N3,P3,N3,N10,0XFF};    //走门路线
//u8 route[100]={B1,N1,P1,N1,B2,N4,N5,N6,P4,N6,N5,N4,N3,P3,0xff};
//u8 route[100]={B2,N4,N5,N6,P4,N6,N5,N4,N3,P3,N3,N4,N5,N6,P4,0xff};
//N13,N18,B5,N19,C6,B7,N22,C9,G1,
//u8 route[100]={N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,B3,N2,P2,0xff};
//`B9,N7,P5,N7,B8,N9,N10,N3,
	
//	B1,N1,P1,N1,B2,N4,N3,P3,N3,N10,N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,
// u8 route[100]={B9,N7,P5,N7,B8,N9,N10,N3,N4,B3,N2,P2,0xff}; // 
//u8 route[100] = {P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,0XFF};   //qqb





u8 route[100] = {N1,P1,N1,B2,N4,N6,S2,N6,P4,N5,N12,0XFF};






//u8 route[100] = {P7,N20,C4,0XFF};
//u8 route[100] = {C3,N9,B9,N7,P5,N7,B8,N9,N10,0XFF};

//u8 route[100] = {N12,N13,P6,N13,N18,B5,N19,0XFF};
//u8 route[100] = {P7,N20,C4,0XFF};
//u8 route[100] = {C3,N9,B9,N7,P5,N7,B8,N9,N10,0XFF};

//C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,
//正常路线
//u8 route[100]={B1,N1,P1,N1,B2,N4,N3,P3,N3,
//				N4,N5,
//				N6,P4,N6,C1,C2,N13,P6,N13,N19,C6,B7,N22,C9,P8,
//				C9,N22,B6,N20,P7,
//				N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,/*B8,N9,N10,
//				N12,N13,C2,C1,N6,N5,N4,B3,N2,P2,*/0XFF};
				
u8 door1route[100]={P3,N3,N8,0XFF};//俩次红灯冲平台三

					
u8 door2route[100]={N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N5,P3,N3,N4,B3,N2,P2,0XFF};//第二次过
//2为第一次红第二次绿
u8 door3route[100]={N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N11,N12,N5,P3,N3,N4,B3,N2,P2,0XFF};//第一次过
//3为第一次绿色过的


u8 door4route[100]={N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N8,N3,N4,B3,N2,P2,0XFF};//出平台三后

u8 door5route[100]={N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,0XFF};
u8 door6route[100]={P3,N3,N4,B3,N2,P2,0XFF};	
u8 door7route[100]={N8,N3,P3,N3,N4,B3,N2,P2,0XFF};//第一个红，第二个黄回来时候第四个红，直接转第三个
u8 door9route[100]={N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,N4,B3,N2,P2,0XFF};
u8 door10route[100]={N12,N13,P6,N13,N18,B5,N19,C6,B7,N22,C9,G1,P8,G1,C9,N22,B6,N20,P7,N20,C4,C8,C7,N14,C3,N9,B9,N7,P5,N7,B8,N9,N10,N3,0XFF};
u8 door11route[100]={N5,P3,N3,N4,B3,N2,P2,0XFF};
u8 door21route[100]={N4,B3,N2,P2,0XFF};
NODESR nodesr;	//运作中间变量
					

//地图初始化
void mapInit()
{
	u8 i=0;
	nodesr.nowNode.nodenum = B1;		//起始点   //N2
	nodesr.nowNode.angle = 0;		//起始角度   //0
	nodesr.nowNode.function = BBridge;	//起始函数   //1
	nodesr.nowNode.speed = 800;//300             
	nodesr.nowNode.step= 80;//60               
	nodesr.nowNode.flag = RIGHT_LINE;    //CLEFT|RIGHT_LINE

    nodesr.flag=0;
	
	
//	nodesr.nowNode.nodenum = N6;		//起始点   //N2
//	nodesr.nowNode.angle = 180;		//起始角度   //0
//	nodesr.nowNode.function =1;	//起始函数   //1
//	nodesr.nowNode.speed = 800;//300             
//	nodesr.nowNode.step= 60;//60               
//	nodesr.nowNode.flag = SLOWDOWN|CLEFT;
	
//	nodesr.nowNode.nodenum = B5;		//起始点   //N2
//	nodesr.nowNode.angle = 180;		//起始角度   //0
//	nodesr.nowNode.function = BHill;	//起始函数   //1
//	nodesr.nowNode.speed = 1200;//300             
//	nodesr.nowNode.step= 70;//60               
//	nodesr.nowNode.flag = STOPTURN|RIGHT_LINE;
	
//	{N4,MUL2MUL|L_follow,180,130,1400,1},
//{N22,DLEFT|RESTMPUZ|SLOWDOWN,0,100,1200,1},
//	{N9,MUL2SING|LiuShui,180,30,1300,1},
//	nodesr.nowNode.nodenum = N9;		//起始点   //N2
//	nodesr.nowNode.angle = 180;		//起始角度   //0
//	nodesr.nowNode.function =1;	//起始函数   //1
//	nodesr.nowNode.speed = 800;//300             
//	nodesr.nowNode.step= 30;//60               
//    nodesr.nowNode.flag = MUL2SING|LiuShui;    //CLEFT|RIGHT_LINE
//	{N5,DLEFT|SLOWDOWN,0,105,1800,1}
//    nodesr.nowNode.nodenum = N5;		//起始点   //N2
//	nodesr.nowNode.angle = 0;		//起始角度   //0
//	nodesr.nowNode.function =1;	//起始函数   //1
//	nodesr.nowNode.speed = 800;//300             
//	nodesr.nowNode.step= 50;//60               
//    nodesr.nowNode.flag = LEFT_LINE|SLOWDOWN;    //CLEFT|RIGHT_LINE

//	{N14,DRIGHT|CRIGHT|RESTMPUZ,-90,200,1000,1}
//	nodesr.nowNode.nodenum = N14;		//起始点   //N2
//	nodesr.nowNode.angle = -90;		//起始角度   //0
//	nodesr.nowNode.function = 1;	//起始函数   //1
//	nodesr.nowNode.speed = 400;//300             
//	nodesr.nowNode.step= 80;//60               
//	nodesr.nowNode.flag = DRIGHT|RESTMPUZ|CRIGHT;    //CLEFT|RIGHT_LINE

//{N3,MUL2SING|SLOWDOWN,-45,10,1000,DOOR},
//	nodesr.nowNode.nodenum = N3;		//起始点   //N2
//	nodesr.nowNode.angle = -45;		//起始角度   //0
//	nodesr.nowNode.function = DOOR;	//起始函数   //1
//	nodesr.nowNode.speed = 300;//300             
//	nodesr.nowNode.step= 40;//60               
//	nodesr.nowNode.flag = MUL2SING|SLOWDOWN;    //CLEFT|RIGHT_LINE

//{P7,NO,0,50,1200,BHM}
//	nodesr.nowNode.nodenum = P7;		//起始点   //N2
//	nodesr.nowNode.angle = 0;		//起始角度   //0
//	nodesr.nowNode.function = BHM;	//起始函数   //1
//	nodesr.nowNode.speed = 300;//300             
//	nodesr.nowNode.step= 45;//60               
//	nodesr.nowNode.flag = NO;    //CLEFT|RIGHT_LINE
//{P3,LEFT_LINE,-1,350,1400,UpStage}
//	nodesr.nowNode.nodenum = P3;		//起始点   //N2
//	nodesr.nowNode.angle = -1;		//起始角度   //0
//	nodesr.nowNode.function = UpStage;	//起始函数   //1
//	nodesr.nowNode.speed = 1000;//300             
//	nodesr.nowNode.step= 300;//60               
//	nodesr.nowNode.flag = LEFT_LINE;    //CLEFT|RIGHT_LINE
//{N4,MUL2MUL|L_follow|LiuShui,180,130,1400,1},
//	nodesr.nowNode.nodenum = N4;		//起始点   //N2
//	nodesr.nowNode.angle = 180;		//起始角度   //0
//	nodesr.nowNode.function = 1;	//起始函数   //1
//	nodesr.nowNode.speed = 500;//300             
//	nodesr.nowNode.step= 80;//60               
//	nodesr.nowNode.flag =MUL2MUL|L_follow|LiuShui;    //CLEFT|RIGHT_LINE
//{N20,CRIGHT|DRIGHT|RESTMPUZ|LEFT_LINE,0,110,1200,1}
//	nodesr.nowNode.nodenum = N20;		//起始点   //N2
//	nodesr.nowNode.angle = 0;		//起始角度   //0
//	nodesr.nowNode.function = 1;	//起始函数   //1
//	nodesr.nowNode.speed = 500;//300             
//	nodesr.nowNode.step= 20;//60               
//	nodesr.nowNode.flag =CRIGHT|DRIGHT|RESTMPUZ|LEFT_LINE;    //CLEFT|RIGHT_LINE

	for(i=0;i<118;i++)			//全地图速度调整
		Node[i].speed*=0.8f;//0.8	    
}
void mapInit1()
{
	nodesr.nowNode.nodenum = B1;		//起始点   //N2
	nodesr.nowNode.angle = 0;		//起始角度   //0
	nodesr.nowNode.function = BBridge;	//起始函数   //1
	nodesr.nowNode.speed = 800;//300             
	nodesr.nowNode.step= 80;//60               
	nodesr.nowNode.flag = RIGHT_LINE;    //CLEFT|RIGHT_LINE
//	
    nodesr.flag=0;   
}

u8 getNextConnectNode(u8 nownode,u8 nextnode) 
{//得到本节点到相邻结点的地址
	unsigned char rest = ConnectionNum[nownode];//这个结点相邻的结点数
	unsigned char addr = Address[nownode];//得到结点的addr
	int i = 0;
	for (i = 0; i < rest; i++) 
	{
		if(Node[addr].nodenum == nextnode)//返回结点地址
		{			
			return addr;
		}
		addr++;
	}
	return 0;
}

u8 deal_arrive()
{				
	register uint8_t lnum = 0, i = 0;
	register uint16_t seed = 0;
	static uint16_t mul2sing = 0, sing2mul = 0;
	getline_error();
	if ((nodesr.nowNode.flag & DLEFT) == DLEFT)  //左半边
	{
		//左边6个灯任意5个亮即可
		if (Scaner.ledNum>=5)
		{
			seed = 0X8000;
			for (i = 0; i<6; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
					return 1;
				seed >>= 1;
			}
			lnum = 0;
		}
	}
	if ((nodesr.nowNode.flag & DRIGHT) == DRIGHT)//右半边
	{    	
		if (Scaner.ledNum >= 5)
		{
			seed = 0X0001;
			for (i = 0; i<6; i++)
			{
				if (Scaner.detail & seed)
					++lnum;
				if (lnum >= 5)
					return 1;
				seed <<= 1;
			}
			lnum = 0;
		}
	}

	if ((nodesr.nowNode.flag & CLEFT) == CLEFT)//左分岔路
	{
		//左边数起第二、第三个灯任意一个亮即可
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && ((Scaner.detail&0x4000)|(Scaner.detail&0x2000)) )//
		 {
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCLEFT) == MCLEFT)//左分岔路
	{
		//左边数起第一个灯亮即可
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x0001) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MCRIGHT) == MCRIGHT)//左分岔路
	{
		//左边数起第一个灯亮即可
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0x8000) )
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & CRIGHT) == CRIGHT)//右分岔路
	{
		 if( (Scaner.ledNum>=4&&Scaner.ledNum<=7) && (Scaner.detail&0xc) )//右起2和3灯亮
		{
			return 1;
		}
	}
	if ((nodesr.nowNode.flag & MORELED) == MORELED)//右分岔路
	{
		 if( (Scaner.ledNum>=5) )//5个灯以上亮
		{
			return 1;
		}
	}
	else if ((nodesr.nowNode.flag & AWHITE) == AWHITE)//全白
	{
		 if((Scaner.ledNum>=10&&(Scaner.detail&0x1FF8)==0x1FF8))
		{
			return 1;
		}
	}
	
	else if ((nodesr.nowNode.flag & MUL2SING) == MUL2SING)//三分岔路
	{
//		 if(((Scaner.ledNum>=4&&Scaner.ledNum<=8)&&(Scaner.detail&0x07E0)==0x07E0))//三线
//		{
//			return 1;
//		}
		if (Scaner.lineNum > 1 || Scaner.ledNum >= 5)
			++mul2sing;
//		if(nodesr.nowNode.nodenum==N9&&nodesr.nextNode.nodenum==B9)
//		{
//			if (mul2sing>=3 && Scaner.lineNum==1&&Scaner.lineNum>=4) //线数目由多变成一条
//			{
//				mul2sing = sing2mul = 0;
//				return 1;
//			}
//		}
//		else
//		{
			if (mul2sing>=3 && Scaner.lineNum==1) //线数目由多变成一条
			{
				mul2sing = sing2mul = 0;
				return 1;
			}
//		}
		
	}
	
	else if ((nodesr.nowNode.flag & MUL2MUL) == MUL2MUL)  //线数目由多条变多条
	{
		if (Scaner.lineNum>1 || Scaner.ledNum>=5)
			++mul2sing;
		if (mul2sing>=3 && Scaner.lineNum==1)
			++sing2mul;
		if (sing2mul>=3 && (Scaner.lineNum>1 || Scaner.ledNum>=5))
		{
			mul2sing = sing2mul = 0;
			return 1;
		}
	}
	
	return 0;
}


void Cross()
{			
	float num = 0;
	static u8 _flag=1;			//结点动作标志位
	if(map.point == 0)
	{
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//获取下一结点
	}
	if(_flag==1)//循迹
	{
		pid_mode_switch(is_Line);
		if(fabsf(motor_all.Distance) >= 0.7*nodesr.nowNode.step) //距离大于70cm就不循迹了
		{
			_flag=0;
			if(((nodesr.nowNode.flag&RESTMPUZ)==RESTMPUZ))//陀螺仪校正
			{
				if ((Scaner.detail & 0X0180) == 0X0180) //如果在最中间位置
				mpuZreset(imu.yaw, nodesr.nowNode.angle);     //获取补偿角Z
				//nodesr.nowNode.flag&=~0x80;
			}
			if( (fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<=10) || (fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<=10))
			{													 		
				motor_all.Cincrement = 20;	  //原来是1
				motor_all.Cspeed = nodesr.nowNode.speed;	
			 }
			else
			{
				motor_all.Cincrement = 20;	//默认加速度 原来是25
				if ((nodesr.nowNode.flag & SLOWDOWN) == SLOWDOWN)
				{
					motor_all.Cspeed = 650;
				}
				else
				{
					if(0.5f*nodesr.nowNode.speed>1000)
					{
						motor_all.Cspeed=1800;// 原来 40 
					}
					else
					{
						motor_all.Cspeed=nodesr.nowNode.speed;// 原来是0.5    0.9当前最好
					}
				}
				
			}
		}
		else//未达到0.7的距离
		{
				motor_all.Cincrement = 25;	//默认加速度  原来是1.5
				motor_all.Cspeed=nodesr.nowNode.speed;// 0.8原来是
		}
		
	}
	else if(_flag==0)//路径超过0.7  判断路口
	{
		map_function(nodesr.nowNode.function);
		if((nodesr.flag&0x04)!=0x04&&deal_arrive()&&(nodesr.flag&0x80)!=0x80&&(nodesr.flag)!=0x20)	//判断是否到达路口
		{
			bofang_zhiding(9);
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//到达路口		
		}	
		if(((nodesr.flag&0x04)!=0x04&&special_arrive==1))//特殊的切换循迹结点
		{
			scaner_set.CatchsensorNum = 0;
			nodesr.flag |= 0x04;	//到达路口
			special_arrive=0;
		}
	}
	if((nodesr.flag&0x04)==0x04)//转弯（已经到达路口）
	{	
		nodesr.flag&=~0x04;		//	清除到达路口标志
		if(route[map.point-1] != 255)	 //route[i]==255代表一整条路线走完
		{		
			
		    //如果下一结点角度与当前结点角度相同，不需要转，不需要减速	    
			if ((fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<10) 
				||(fabs(need2turn(nodesr.nowNode.angle,nodesr.nextNode.angle))<10)
				||(nodesr.nextNode.flag&NOTURN)==NOTURN)
			{														 		
				_flag=1;
			}
			else //需要转
			{	                
				if(nodesr.nowNode.flag&STOPTURN)    //STOPTURN标志位待变原地转
				{	    	
					num = motor_all.Distance;
				    if(nodesr.nowNode.nodenum==N13)//小角度
					{
						while(motor_all.Distance-num<6)
						{
							vTaskDelay(2);
						}
					}
					else{//90度
						while(motor_all.Distance-num<14)
						{
							vTaskDelay(2);
						}
					}
//					if(nodesr.nowNode.nodenum==)
					angle.AngleT = nodesr.nextNode.angle;  //转绝对角度
					pid_mode_switch(is_Turn);	
					while(fabs(nodesr.nextNode.angle-getAngleZ())>2)
					{
						vTaskDelay(2);
					}		
				}
				else if(nodesr.nowNode.flag&DRIFT)
				{
					pid_mode_switch(is_Gyro);
					struct PID_param origin_parm=gyroG_pid_param;
					gyroG_pid_param.kp=0.95;
					float origin_speedMax=motor_all.GyroG_speedMax;
					
					num = motor_all.Distance;
					motor_all.Gspeed=motor_all.Cspeed;
					motor_all.GyroG_speedMax=0.45f*motor_all.Gspeed;
					motor_all.Gincrement=20;
					angle.AngleG=nodesr.nextNode.angle;
//					if(nodesr.nowNode.nodenum==N7)
//					{
//						GyroP_pid_param.kp=1.65;
//					}
					while(fabsf(nodesr.nextNode.angle-getAngleZ()) >2)
					{
//						Drift(motor_all.Pspeed,nodesr.nextNode.angle);
						if(Scaner.lineNum==1&&Scaner.detail&0x7E0&&(fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.2f))
						{
							break;
						}
						vTaskDelay(2);
					}
					gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
					gyroG_pid_param=origin_parm;
					motor_all.GyroG_speedMax=origin_speedMax;
				}
				else if(nodesr.nowNode.flag&L_follow)//左循迹加大kp的转弯，适用小角度，多线干扰陀螺仪摆尾位置不合适
				{
					float original_line_pid = line_pid_param.kp;
					float max = line_pid_param.outputMax;
					line_pid_param.kp = 40;   //增大KP辅助转弯
					line_pid_param.outputMax = 0.6f*motor_all.Cspeed;
					nodesr.nowNode.flag|=LEFT_LINE;
					angle.AngleG = nodesr.nextNode.angle;
                   				
					while(fabs(need2turn(angle.AngleG,getAngleZ()))>2)
					{   //角度达到要求
						vTaskDelay(2);
						if(Scaner.lineNum==1&&Scaner.detail&0xff0&&(fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.5f))
						{  
									break;
						}
					}
					nodesr.nowNode.flag&=(~LEFT_LINE);//取消左循迹标志位
					line_pid_param.kp = original_line_pid;  //恢复正常
					line_pid_param.outputMax =  max;
				}
				else if(nodesr.nowNode.flag&R_follow)//左循迹加大kp的转弯，适用小角度，多线干扰陀螺仪摆尾位置不合适
				{
					float original_line_pid = line_pid_param.kp;
					float max = line_pid_param.outputMax;
					float num=motor_all.Distance;
					if(nodesr.nowNode.nodenum==N4)
					{
						while(motor_all.Distance-num<5)
						{
							vTaskDelay(2);
						}
					}
					line_pid_param.kp = 40;   //增大KP辅助转弯
					line_pid_param.outputMax = 0.6f*motor_all.Cspeed;
					nodesr.nowNode.flag|=RIGHT_LINE;
					angle.AngleG = nodesr.nextNode.angle;
                   				
					while(fabs(need2turn(angle.AngleG,getAngleZ()))>2)
					{   //角度达到要求
						vTaskDelay(2);
						if(Scaner.lineNum==1&&Scaner.detail&0xff0&&(fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,getAngleZ()))*0.5f))
						{  
									break;
						}
					}
					nodesr.nowNode.flag&=(~RIGHT_LINE);//取消左循迹标志位
					line_pid_param.kp = original_line_pid;  //恢复正常
					line_pid_param.outputMax =  max;
				}
				else		//差速转
				{	
					if(fabs(need2turn(getAngleZ(),nodesr.nextNode.angle))<100)//需要转动小于60度
					{
						pid_mode_switch(is_Gyro);
						struct PID_param origin_parm=gyroG_pid_param;
						gyroG_pid_param.kp=0.95;
						float origin_speedMax=motor_all.GyroG_speedMax;
						
						num = motor_all.Distance;
						motor_all.Gspeed=motor_all.Cspeed;
						motor_all.GyroG_speedMax=0.45f*motor_all.Gspeed;
						motor_all.Gincrement=20;
						angle.AngleG=nodesr.nextNode.angle;
	//					if(nodesr.nowNode.nodenum==N7)
	//					{
	//						GyroP_pid_param.kp=1.65;
	//					}
						if(nodesr.nowNode.nodenum==N2)
						{
							while(motor_all.Distance-num<10)
							{
								vTaskDelay(2);
							}
						}
						while(fabsf(nodesr.nextNode.angle-getAngleZ()) >2)
						{
	//						Drift(motor_all.Pspeed,nodesr.nextNode.angle);
							if(Scaner.lineNum==1&&Scaner.detail&0x7E0&&(fabs(need2turn(angle.AngleG,getAngleZ()))<fabs(need2turn(angle.AngleG,nodesr.nowNode.angle))*0.2f))
							{
								break;
							}
							vTaskDelay(2);
						}
						gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
						gyroG_pid_param=origin_parm;
						motor_all.GyroG_speedMax=origin_speedMax;
					}		
					else
					{
						num=motor_all.Distance;
						if(nodesr.nowNode.nodenum==N8&&nodesr.nextNode.nodenum==N12)
						{
							while(motor_all.Distance-num<10)//超大角度 一般提前识别到路口
							{
								vTaskDelay(2);
							}
						}
		                else
						{
							while(motor_all.Distance-num<18)//超大角度 一般提前识别到路口
							{
								vTaskDelay(2);
							}
						}
						
						angle.AngleT = nodesr.nextNode.angle;  //转绝对角度
						pid_mode_switch(is_Turn);	
						while(fabs(nodesr.nextNode.angle-getAngleZ())>2)
						{
							vTaskDelay(2);
						}									
					}
				}
			}
			//转完后进入循迹模式，更新结点，清空编码器值
			_flag=1;
			buzzer_off();
			pid_mode_switch(is_Line);	
			nodesr.nowNode=nodesr.nextNode;	//更新结点
			motor_all.Cspeed=nodesr.nowNode.speed;
			nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];//获取下一结点	
			scaner_set.EdgeIgnore=0;
			scaner_set.CatchsensorNum=0;
			//路程记录清零
		     encoder_clear();
			nodesr.flag&=~0x04;		//	清除到达路口标志
		} 
		else//如果路线走完
		{		
			motor_all.Cspeed=0;
			CarBrake();
			_flag=1;
			map.routetime+=1;
		}	
	}	
	if(nodesr.flag&0x20)	//如果打到门是关的情况
	{				
		_flag=1;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
		if(nodesr.nextNode.nodenum!=N8)
		nodesr.nextNode.function = DOOR;
		nodesr.flag&=~0x20;
	}
	if(nodesr.flag&0x80)//如果打到门是开着
	{
		_flag=1;
		nodesr.nextNode	= Node[getNextConnectNode(nodesr.nowNode.nodenum,route[map.point++])];		//对下一结点进行更新
//		nodesr.nextNode.function = 1;
		nodesr.flag&=~0x80;
		pid_mode_switch(is_Gyro);
		angle.AngleG=getAngleZ();
		motor_all.Gspeed=600;
		float num=motor_all.Distance;
		while(motor_all.Distance-num<6)
		{
			vTaskDelay(2);
		}
		pid_mode_switch(is_Line);
		motor_all.Cspeed=nodesr.nowNode.speed;
	}
}
	

void map_function(u8 fun)
{
	switch(fun)
	{
		case 0:break;
		case 1:break;		//寻线
		case UpStage      :Stage();break;//平台
		case BBridge  	  :Barrier_Bridge(150,30);break;//长桥 长度，速度
		case BHill	  	  :Barrier_Hill(1);break;//楼梯
		case LBHill       :Barrier_Hill(2); break;  //双楼梯
		case SM           :Sword_Mountain();break;//刀山
		case View	      :view();break;//景点 后转
		case View1        :view1();break;//景点 直退
		case BACK         :back();break;
		case BSoutPole	  :South_Pole(205);break;//南极
		case QQB	      :QQB_1();break;//跷跷板
		case BLBS         :Barrier_WavedPlate(40);break;//短减速板 速度，长度
		case BLBL	      :Barrier_WavedPlate(150);break;//长减速板 速度，长度//130改到150
		case DOOR	      :door();break;//打门
		case BHM          :Barrier_HighMountain(600); break;//上珠峰
		case Scurve	  	  :S_curve();  break;
		case IGNORE       :ignore_node(); break;  //忽略该节点
		case UNDER        :undermou();break;
		case Special_node :Special_Node();break;
		default:break;		
	}
}


