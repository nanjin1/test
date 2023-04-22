#include "turn.h"
#include "imu_task.h"
#include "pid.h"
#include "speed_ctrl.h"
#include "math.h"
#include "map.h"
#include "motor_task.h"
#include "uart.h"
volatile struct Angle angle = {0, 0};

//�����ǰ�Ƕ���Ŀ��Ƕȵ���С�н�
float need2turn(float nowangle,float targetangle)
{			
	float need2Turn;		

	need2Turn=targetangle-nowangle;		//ʵ������ת�ĽǶ�
	if(need2Turn>180)	need2Turn -= 360;
  else if(need2Turn<-180)	need2Turn += 360;
	
  return need2Turn;		
}

//��������У׼  
void mpuZreset(float sensorangle ,float referangle)
{	
	imu.compensateZ=need2turn(sensorangle,referangle);
}


//��ȡZ�Ƕ�
float getAngleZ(void) 
{
	float targetangle;
	targetangle = imu.yaw + imu.compensateZ;

	if(targetangle>180)	targetangle -= 360;
	else if(targetangle<-180)	targetangle += 360;
	
	return targetangle;
}
//ƽ̨ת��
uint8_t Stage_turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//�ٽ紦��
	if(Angle>180)	Angle -= 360;
	else if(Angle<-180)	Angle += 360;
	
	now_angle = getAngleZ();
	if (fabsf(Angle-now_angle) < 1)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}
	
	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);
	
	if(GTspeed >= motor_all.GyroT_speedMax) 
		GTspeed = motor_all.GyroT_speedMax;
	else if(GTspeed <= -motor_all.GyroT_speedMax) 
		GTspeed = -motor_all.GyroT_speedMax;

	if(nodesr.nowNode.nodenum == P1)
	{
		motor_all.Lspeed = GTspeed; 
		motor_all.Rspeed = -1.2f*GTspeed;
	}
	else if(nodesr.nowNode.nodenum==P8)
	{
		motor_all.Lspeed = 1.3f*GTspeed; 
		motor_all.Rspeed = -GTspeed;
	}
	else if(nodesr.nowNode.nodenum==P2)
	{
		motor_all.Lspeed = 1.25f*GTspeed; 
		motor_all.Rspeed = -GTspeed;
	}

	else 
	{
		motor_all.Lspeed = GTspeed; 
		motor_all.Rspeed = -GTspeed;
	}
	return 0;
}

/*****************************************************************************
������д�˼��������ڣ���������7-19
�������ܣ�������Z��ת�Ƕȣ�ת��ԽǶ�
���룺Ҫת�ĽǶ� ���ٶ�
�������
*****************************************************************************/
void Turn_Angle_Relative(float Angle1)//��180����-180,�ٶȱ��������ģ�
{
	float Turn_Angle_Before = 0, Turn_Angle_Targe = 0;
	
	Turn_Angle_Before = getAngleZ();//��ȡ��ǰ�ĽǶ�//@@@@@
	Turn_Angle_Targe = Turn_Angle_Before+Angle1;//Ŀ��Ƕ���Ϊ��������
	/*******************��������ٽ�״̬����Ŀ��Ƕ�ת��Ϊ��������******180 0 -180*************/
    if(Turn_Angle_Targe>180){				
    	Turn_Angle_Targe = Turn_Angle_Targe-360;
    }
    else if(Turn_Angle_Targe<-180){
    	Turn_Angle_Targe=Turn_Angle_Targe+360;
    }
	
	angle.AngleT = Turn_Angle_Targe;
	pid_mode_switch(is_Turn);  //����ת��
}

/*****************************************************************************
�������ܣ�������Z����PIDԭ��ת�Ƕ�
					��Ҫת���ĽǶ�(���ԽǶ�)
					�÷���Target = ?,PIDMode = is_Turn,while(fabs(getAngleZ()-Target)<1);
*****************************************************************************/
uint8_t Turn_Angle(float Angle)	
{
	float GTspeed, now_angle;
	
	//�ٽ紦��
	if(Angle>180)	Angle -= 360;
	else if(Angle<-180)	Angle += 360;
	
	now_angle = getAngleZ();
	if (fabsf(Angle-now_angle) < 1)
	{
		motor_all.Lspeed = motor_all.Rspeed = 0;
		gyroT_pid.integral = 0;
		gyroT_pid.output = 0;
		return 1;
	}
	
	gyroT_pid.measure = need2turn(now_angle, Angle);
	gyroT_pid.target = 0;

	GTspeed = positional_PID(&gyroT_pid, &gyroT_pid_param);
	
	if(GTspeed >= motor_all.GyroT_speedMax) 
		GTspeed = motor_all.GyroT_speedMax;
	else if(GTspeed <= -motor_all.GyroT_speedMax) 
		GTspeed = -motor_all.GyroT_speedMax;

	motor_all.Lspeed = GTspeed; 
	motor_all.Rspeed = -GTspeed;
	
	return 0;
}


/*****************************************************************************
�������ܣ�ԭ����ת360��
		˵�������ö����ת���趨��ֵ������Ŀ��ֵǰ���и���
*****************************************************************************/
void Turn_Angle360(void)
{
	/*if(nodesr.nowNode.nodenum == P7||nodesr.nowNode.nodenum == P8)
	{
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10);//4
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10);//4
		Turn_Angle_Relative(120);//120
		while(fabs(angle.AngleT - getAngleZ())>2);//4
	}*/
	
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10)//4
		{
			vTaskDelay(1);
		}
		Turn_Angle_Relative(130);//160
		while(fabs(angle.AngleT - getAngleZ())>10)//4
		{
			vTaskDelay(1);
		}	
		Turn_Angle_Relative(120);//120
		while(fabs(angle.AngleT - getAngleZ())>2)//4
		{
			vTaskDelay(1);
		}

}

/*****************************************************************************
�������ܣ�������Z����ƽ������
*****************************************************************************/
uint8_t runWithAngle(float angle_want,float speed)
{
	float GGspeed, now_angle;
	
	now_angle = getAngleZ();
	
//	if (fabsf(angle_want-now_angle) < 1)
//	{
//		motor_all.Lspeed = motor_all.Rspeed = speed;
//		return 0;
//	}
	
	gyroG_pid.measure = need2turn(now_angle, angle_want);  
	gyroG_pid.target = 0;
	
	GGspeed = positional_PID(&gyroG_pid, &gyroG_pid_param);
	
	if(GGspeed >= motor_all.GyroG_speedMax) 
		GGspeed = motor_all.GyroG_speedMax;
	else if(GGspeed <= -motor_all.GyroG_speedMax) 
		GGspeed = -motor_all.GyroG_speedMax;

	motor_all.Lspeed = speed+GGspeed*speed/50;
	motor_all.Rspeed = speed-GGspeed*speed/50;	
//	printf("L=%f  R=%f,GGSPEED=%f,kp=%f,measure=%f\r\n",motor_all.Lspeed,motor_all.Rspeed,GGspeed,gyroG_pid_param.kp,gyroG_pid.measure);
//	printf("measure=%f\r\n",gyroG_pid.measure);
	return 1;
}

/*****************************************************************************
�������ܣ�����ת����û�������pid�ɣ�
*****************************************************************************/
void AdCircle(float speed, float radius) 
{
	motor_all.Lspeed = speed - radius;
	motor_all.Rspeed = speed + radius;	
}
uint8_t Drift(float speed,float Angle)
{
	float GPspeed, now_angle;
	
	//�ٽ紦��
	if(Angle>180)	Angle -= 360;
	else if(Angle<-180)	Angle += 360;
	
	now_angle = getAngleZ();
	if (fabsf(Angle-now_angle) < 1)
	{
		motor_all.Lspeed = motor_all.Rspeed = speed;
		GyroP_pid.integral = 0;
		GyroP_pid.output = 0;
		return 1;
	}
	GyroP_pid.measure = need2turn(now_angle, Angle);
	GyroP_pid.target = 0;

	GPspeed = positional_PID(&GyroP_pid, &GyroP_pid_param);
	motor_all.GyroP_speedMax=speed*0.45f;
	if(GPspeed >= motor_all.GyroP_speedMax) 
		GPspeed = motor_all.GyroP_speedMax;
	else if(GPspeed <= -motor_all.GyroP_speedMax) 
		GPspeed = -motor_all.GyroP_speedMax;
//    if(GPspeed<0)
//	{
//		motor_all.Lspeed = speed+GPspeed*speed/50;
//		motor_all.Rspeed = speed-GPspeed*speed*1.3f/50;	
//	}
//	else{
		motor_all.Lspeed = speed+GPspeed*speed/50;
		motor_all.Rspeed = speed-GPspeed*speed/50;	
//	}
	
	return 0;


}




