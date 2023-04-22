#ifndef __task_create_h__
#define __task_create_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//��ʼ����
extern TaskHandle_t Start_handler ; //���忪ʼ������
void Start_task(void *pvParameters);//����������
#define Start_size  1024   //�����ջ��С
#define Start_task_priority 32  //�������ȼ�


//��������
extern TaskHandle_t main_handler ; //��������������
void main_task(void *pvParameters);//����������
#define main_size  1024*5  //�����ջ��С
#define main_task_priority 12  //�������ȼ�



void Start_task_create(void);
void main_task_create(void);

#endif
