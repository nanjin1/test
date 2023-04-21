#include "motor_task.h"
#include  "encoder.h"
#include "motor.h"
#include "uart.h"
#include "speed_ctrl.h"
#include "pid.h"
#include "turn.h"
#include "scaner.h"
#include "bsp_linefollower.h"
#include "sin_generate.h"
#include "bsp_buzzer.h"
#include "openmv.h"
#include "map.h"
#include "QR.h"
TaskHandle_t motor_handler;
int dirct[4] = {-1,1,-1,-1};  
volatile uint8_t PIDMode;
uint8_t line_gyro_switch = 0;

#define Speed_Bias_Up 10
#define Speed_Bias_Down 10

int motor1;
void motor_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //��ȡϵͳ����
	static uint8_t mouse = 0;    //С����

	while(1){
			get_motor_speed();
			if(infrare_open)
			{ //��������
				 get_Infrared();
			}
		   motor_all.encoder_avg = (Speed[0] + Speed[1] + Speed[2] + Speed[3]) / 4;
			//�־�6.8cm
//			printf("num=%d,out=%d\r\n",pulse_num[0],pulse_out[0]);
//			printf("%f\r\n",motor_all.encoder_avg);
			motor_all.Distance += motor_all.encoder_avg * 5.0f;
//            printf("pid=%d\r\n",PIDMode);//
//			printf("%f,%f,%f\r\n",imu.pitch,imu.roll,imu.yaw);
//			printf("%f\r\n",motor_all.Distance);
		//��������ƽ��->ѭ��
			if (line_gyro_switch == 1)    //�����line_gyro_switch����PIDMODE�л�������������ı�־λ
			{
				line_pid_obj = gyroG_pid;
				TC_speed = TG_speed;
				gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
				TG_speed = (struct Gradual){0,0,0};
				line_gyro_switch = 0;
			}
			//ѭ��->��������ƽ��
			else if (line_gyro_switch == 2)
			{
				gyroG_pid = line_pid_obj;
				TG_speed = TC_speed;
				line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
				TC_speed = (struct Gradual){0,0,0};
				line_gyro_switch = 0;
			}
			else
			{
				if (PIDMode == is_Line)
				{
					    getline_error();
						gradual_cal(&TC_speed, motor_all.Cspeed, motor_all.Cincrement);
						Go_Line(TC_speed.Now);
				}
				else
					motor_all.Cspeed = 0;
				
				//ת��PID����
				if (PIDMode == is_Turn)	
				{
					if(nodesr.nowNode.function == UpStage || nodesr.nowNode.function == BSoutPole || nodesr.nowNode.function == BHM )
					{
						if (Stage_turn_Angle(angle.AngleT))
						{
							gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0};
						}
					}
					else if (Turn_Angle(angle.AngleT))
					{
						gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0};
					}
				}
				//��ƽ��PID����
				if (PIDMode == is_Gyro)
				{ 
					gradual_cal(&TG_speed, motor_all.Gspeed, motor_all.Gincrement);	
					runWithAngle(angle.AngleG, TG_speed.Now);
				}
				else
					motor_all.Gspeed = 0;
			}
//			printf("%f\r\n",imu.yaw);
//			printf("%d\r\n",color_R);
//			printf("%d\r\n",BW_add);
//			printf("name=%d\r\n",nodesr.nextNode.nodenum);
//    		runWithAngle(0,500);
//			printf("%d��z=%f\r\n",PIDMode,getAngleZ());
//            printf("z=%f,pid=%d\r\n",getAngleZ(),PIDMode);
			motor_L0.target = motor_L1.target = motor_all.Lspeed;
			motor_R0.target = motor_R1.target = motor_all.Rspeed;
			mouse++;
			if(mouse>100){
						mouse =0;
//						LED_twinkle();
				HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
			}
	
			incremental_PID(&motor_L0, &motor_pid_paramL0);
			incremental_PID(&motor_L1, &motor_pid_paramL1);
			incremental_PID(&motor_R0, &motor_pid_paramR0);
			incremental_PID(&motor_R1, &motor_pid_paramR1);

	
//			TIM1->CCR2 = 0; TIM1->CCR4 = 6000;
//			TIM1->CCR1 = 0; TIM1->CCR3 = 6000;
//			TIM2->CCR4 = 0; TIM2->CCR2 = 6000;
//			TIM2->CCR1 = 0; TIM2->CCR3 = 6000;
			if(motor_all.Lspeed==0&&motor_all.Rspeed==0&&
			 ((motor_L0.measure+motor_L1.measure+motor_R0.measure+motor_R1.measure)<=50))//����ʱ�ٶȺ͵���50���0
			{
				motor_pid_clear();
			}
			
//			motor_L0.output = 3000;
//			motor_L1.output = 3000;
//			motor_R0.output = 3000;
//			motor_R1.output = 3000;
			

//			motor_set_pwm(1, 5000);
//			motor_set_pwm(2, 5000);
//			motor_set_pwm(3, 5000);
//			motor_set_pwm(4, 5000);

			motor_set_pwm(1, (int32_t)motor_L0.output*0.1);
			motor_set_pwm(2, (int32_t)motor_L1.output*0.1);
			motor_set_pwm(3, (int32_t)motor_R0.output*0.1);
	    motor_set_pwm(4, (int32_t)motor_R1.output*0.1);
//			
//		printf("%d,%d,%d,%d\r\n",(int)motor_L0.measure,(int)motor_L1.measure,(int)motor_R0.measure,(int)motor_R1.measure);
//		printf("%d,%d\r\n",(int)motor_R1.measure,(int)motor_R1.target);
//		printf("%5d  %5d %5d %5d\r\n",(int)direction[0],(int)high_time[1],(int)direction[2],(int)direction[3]);
		vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//��������5ms // INCLUDE_vTaskDelayUntil 1
	
   }
}

//PID���񴴽�
void motor_task_create(void){
	  xTaskCreate((TaskFunction_t ) motor_task,//������
	               (const char *)"motor_task",	  //��������
								 (uint32_t) motor_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) motor_task_priority, //��������ȼ�
								(TaskHandle_t *)&motor_handler ); //������
							 }
void pid_mode_switch(uint8_t target_mode)
{
	switch (target_mode)
	{
		case is_Turn: {
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
		
		case is_Line: {
			if (PIDMode == is_Gyro)  //����ƽ���л���ѭ��
			{
				line_gyro_switch = 1;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Gyro: {
			if (PIDMode == is_Line)  //��ѭ���л�����ƽ��
			{
				line_gyro_switch = 2;
			}
			else
			{
				gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			}
			break;
		}
		
		case is_Free: {
			break;
		}
		
		case is_No: {
			line_pid_obj = (struct P_pid_obj){0,0,0,0,0,0,0};
			TC_speed = (struct Gradual){0,0,0};
			gyroG_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			TG_speed = (struct Gradual){0,0,0};
			gyroT_pid = (struct P_pid_obj){0,0,0,0,0,0,0};
			break;
		}
		case is_sp :
		{
			break;
		}
	}
	PIDMode = target_mode;
}

void get_motor_speed()
{
	    
			motor_L0.measure = (float)Speed[0];
		
			motor_L1.measure = (float)Speed[1];
		
			motor_R0.measure = (float)Speed[2];
		
			motor_R1.measure = (float)Speed[3];
}
