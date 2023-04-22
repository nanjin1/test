#include "task_create.h"
TaskHandle_t Start_handler; //���忪ʼ������
TaskHandle_t main_handler; //��������������

/*
��ʼ���񴴽�����
*/
void Start_task_create(void){
	  xTaskCreate((TaskFunction_t ) Start_task,//������
	               (const char *)"Start_task",	  //��������
								 (uint32_t) Start_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) Start_task_priority, //��������ȼ�
								(TaskHandle_t *)&Start_handler ); //������
}


//�������񴴽�
void main_task_create(void){
	  xTaskCreate((TaskFunction_t ) main_task,//������
	               (const char *)"main_task",	  //��������
								 (uint32_t) main_size,    //�����ջ��С
								 (void* )NULL,                  //���ݸ����������ָ�����
								 (UBaseType_t) main_task_priority, //��������ȼ�
								(TaskHandle_t *)&main_handler ); //������
}
