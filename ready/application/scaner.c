#include "scaner.h"
#include "map.h"
#include "math.h"
#include "turn.h"
#include "stdio.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "bsp_linefollower.h"
#include "motor.h"

#define LINE_SPEED_MAX 50
#define Speed_Compensate   5
#define BLACK 0								//ѭ����
#define WHLITE 1							//ѭ����

const float line_weight[16] = {-3.1,-2.4,-1.8,-1.3,-0.9,-0.6,-0.4,-0.1,0.1,0.4,0.6,0.9,1.3,1.8,2.4,3.1};//0.3
//���ҵ���
volatile struct Scaner_Set scaner_set = {0,0};

//float CatchsensorNum = 0;	//Ѱ��ʱ��Ŀ�괫����λ�ã�һ�������Ϊ0��΢΢���õ�
volatile SCANER Scaner;


#define Line_color WHLITE
//ѭ���� 1234578 87654321

void Go_Line(float speed){
	float Fspeed;				//����PID�����Ľ��
					
	line_pid_obj.measure = Scaner.error;  							//��ǰѭ�������ڵ�λ�ã�������-7��0��0��7
	line_pid_obj.target = scaner_set.CatchsensorNum;		//Ŀ��
//	printf("error=%f\r\n",line_pid_obj.measure);
	Fspeed = positional_PID(&line_pid_obj, &line_pid_param); //����λ��PID����
	
	if (Fspeed>=LINE_SPEED_MAX) 
		Fspeed = LINE_SPEED_MAX;
	else if (Fspeed<=-LINE_SPEED_MAX) 
		Fspeed = -LINE_SPEED_MAX;
	
	Fspeed *= fabsf(speed)/50;
//	printf("%f\r\n",Fspeed);
	
	
	motor_all.Lspeed = speed-Fspeed;
	motor_all.Rspeed = speed+Fspeed;
	
}
uint8_t getline_error()//��ø��²�ͬѲ��ģʽ�µ����ֵ
{
	get_detail();//��ȡѲ��ֵ
	if(Line_Scan(&Scaner, Lamp_Max, scaner_set.EdgeIgnore))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void get_detail()
{
	uint32_t data = 0;
	//�������Ʒ���0
		//��ߵĵ�
	//��������   1 2 3 4 5 6 7 8
	if(Line_color){
	data = 0xffff;//����
    data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left1,GPIO_Left1_P)<<15); //��ͬ���1.��ͬ���0
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left2,GPIO_Left2_P)<<14);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left3,GPIO_Left3_P)<<13);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left4,GPIO_Left4_P)<<12);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left5,GPIO_Left5_P)<<11);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left6,GPIO_Left6_P)<<10);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left7,GPIO_Left7_P)<<9);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left8,GPIO_Left8_P)<<8);
	//�ұ�
	//���ҵ���   8 7 6 5 4 3 2 1
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right1,GPIO_Right1_P)<<0);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right2,GPIO_Right2_P)<<1);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right3,GPIO_Right3_P)<<2);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right4,GPIO_Right4_P)<<3);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right5,GPIO_Right5_P)<<4);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right6,GPIO_Right6_P)<<5);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right7,GPIO_Right7_P)<<6);
	data^=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right8,GPIO_Right8_P)<<7);
}
	else{
	data = 0x0;//����
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left1,GPIO_Left1_P)<<15);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left2,GPIO_Left2_P)<<14);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left3,GPIO_Left3_P)<<13);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left4,GPIO_Left4_P)<<12);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left5,GPIO_Left5_P)<<11);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left6,GPIO_Left6_P)<<10);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left7,GPIO_Left7_P)<<9);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Left8,GPIO_Left8_P)<<8);
	//�ұ�
	//���ҵ���   8 7 6 5 4 3 2 1
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right1,GPIO_Right1_P)<<0);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right2,GPIO_Right2_P)<<1);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right3,GPIO_Right3_P)<<2);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right4,GPIO_Right4_P)<<3);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right5,GPIO_Right5_P)<<4);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right6,GPIO_Right6_P)<<5);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right7,GPIO_Right7_P)<<6);
	data|=((uint16_t)HAL_GPIO_ReadPin(GPIO_Right8,GPIO_Right8_P)<<7);
	}
	Scaner.detail = data;
//	printf("num=%f\r\n",scaner_set.CatchsensorNum);
}
//ѭ��ɨ��
uint8_t Line_Scan(SCANER *scaner, unsigned char sensorNum, int8_t edge_ignore)
{
	float error = 0;
	u8 linenum=0;//��¼�ߵ���Ŀ
	u8 lednum=0;
	int8_t lednum_tmp = 0;
							//��ö�����Ѳ��ֵ
	for(uint8_t i=0;i<sensorNum;i++) 		//��С�������������������������������
	{								//linenum������¼�ж������ߣ�line������¼�ڼ����ߡ�
		if((scaner->detail&(0x1<<i))) 
		{
			lednum++;
			if(!(scaner->detail&(1<<(i+1)))) 
				++linenum;			//�ȶ�ȡ��������������������⵽��1��Ϊ0��Ϊһ����
		}
	}
	scaner->lineNum = linenum;			
	scaner->ledNum=lednum;
////		for (uint8_t i=0; i<sensorNum; i++)
////		{
////			lednum_tmp += (scaner->detail>>(sensorNum-i-1))&0X01;//��¼�����
////			error += ((scaner->detail>>(sensorNum-i-1))&0X01) * line_weight[i];
////		}
	if ((nodesr.nowNode.flag & LEFT_LINE) == LEFT_LINE)  //��ѭ��    
	{
		for (uint8_t i=0; i<sensorNum; i++)
		{
			lednum_tmp += (scaner->detail>>(sensorNum-i-1))&0X01;//��¼�����
			error += ((scaner->detail>>(sensorNum-i-1))&0X01) * line_weight[i];
			if ((scaner->detail>>(sensorNum-i-1)) & 0X01)		//����ǰ���
				if (!((scaner->detail>>((sensorNum-i-2)))&0x01))		//��һ���Ʋ��ǰ�
					break;		//�˳�
		}
	}
	else if ((nodesr.nowNode.flag & RIGHT_LINE) == RIGHT_LINE)  //��ѭ��
	{
		for (uint8_t i=0; i<sensorNum; i++)
		{
			lednum_tmp += (scaner->detail>>i)&0X01;
			error += ((scaner->detail>>i)&0X01) * line_weight[sensorNum-i-1];
			if ((scaner->detail>>i) & 0X01)
				if (!((scaner->detail>>(i+1))&0x01))
					break;
		}
	}
	else if((nodesr.nowNode.flag&LiuShui)==LiuShui)
	{
		uint8_t flag=0;
		float error1=0;
		float error2=0;
		uint8_t lednum_temp1=0;
		uint8_t lednum_temp2=0;
		for(uint8_t i=0;i<sensorNum;i++)
		{
			if(flag==0)//�ұ���
			{
				error1+= ((scaner->detail>>i)&0X01) * line_weight[sensorNum-i-1];
				lednum_temp1+= (scaner->detail>>i)&0X01;
			}
			else if(flag==1)//�����
			{
				error2+= ((scaner->detail>>i)&0X01) * line_weight[sensorNum-i-1];
				lednum_temp2+= (scaner->detail>>i)&0X01;
			}
			if ((scaner->detail>>i) & 0X01)
			{
				if (!((scaner->detail>>(i+1))&0x01))
				{
					flag=1;
				}
			}
		}
		if(error1!=0)
			error1/=lednum_temp1;
		else
			error1=0;
		if(lednum_temp2==0)//ֻ��һ����
			error2=0;
		else
			error2/=lednum_temp2;
		if(lednum>=6)
		{
			Scaner.error = 0;
			return 0;
		}
		else
		{
			if(fabs(error2)>fabs(error1)|lednum_temp2==0)
			{
				Scaner.error = error1;
				return 0;
			}
			else if((lednum_temp2!=0)&(fabs(error2)<fabs(error1)))
			{
				Scaner.error = error2;
				return 0;
			}
		}
		
	}
	else//����������
	{
		for(uint8_t i= edge_ignore; i<sensorNum - edge_ignore; i++) 
		{
				lednum_tmp += (scaner->detail>>(sensorNum-1-i))&0X01;
				error += ((scaner->detail>>(sensorNum-1-i))&0X01) * line_weight[i];
		}
		if(lednum_tmp>=4)
		{
			Scaner.error=0;
			return 0;
		}       		
	}
	if(lednum==0|lednum_tmp==0)
	{
		error=0;
	}
	else
	{
		error/=(float)lednum_tmp;		//ȡƽ��
	}
	Scaner.error = error;
	return 0;
}

