#ifndef __motor_task_h__
#define __motor_task_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"
//��ʼ����
extern TaskHandle_t motor_handler ; 				//���忪ʼ������
void motor_task(void *pvParameters);//����������
#define motor_size  1024   					//�����ջ��С
#define motor_task_priority 10 			//�������ȼ�
#define PI  3.1415926535f
void motor_task_create(void);


enum PID_Mode {
	is_No = 0,  //�ر����в���
	is_Free,   //�����л�ǰ��״̬1
	is_Line,   //ѭ��22
	is_Turn,   //ת��3
	is_Gyro,   //��ƽ��4
	is_sp
};


void pid_mode_switch(uint8_t target_mode);
void get_motor_speed(void);
#endif
