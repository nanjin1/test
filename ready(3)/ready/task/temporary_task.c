#include "temporary_task.h"
#include "task_create.h"
#include "delay.h"
#include "usmart.h"
#include "pid.h"
#include "motor.h"
#include "motor_task.h"
#include "encoder.h"
#include "uart.h"
#include "rudder_control.h"
#include "imu_task.h"
#include "bsp_led.h"
#include "turn.h"
#include "map.h"
#include "openmv.h"
#include "QR.h"
/*
��ʼ����
*/
void Start_task(void *pvParameters){
		taskENTER_CRITICAL();  	//�����ٽ���
	//   	user_init();  
	    main_task_create();            //������������
		motor_task_create();
	    vTaskDelete(Start_handler); //ɾ����ʼ����
	  	taskEXIT_CRITICAL();            //�˳��ٽ���
}

/*****************************************************************************
�������� GET_free_RAM
�������ܣ���������ʣ���ջ��С ���Ҵ�ӡ  
�βΣ�������ľ��      ������NULL����Ϊ������Ķ�ջ 
ע�⣺INCLUDE_uxTaskGetStackHighWaterMark 1    ����ʹ��
*******************************************************************************/
void GET_free_RAM(TaskHandle_t xTask){
//	printf("RAM = %d\r\n",(int32_t)uxTaskGetStackHighWaterMark(xTask));
		vTaskDelay(500);
}
/*****************************************************************************
�������� user_init
�������ܣ���ʼ�����裬�ṹ��� 
*******************************************************************************/
void user_init(void){
	Encoder_init();
	uart_init(115200);  	//��ʼ���ض��򴮿�

//	usmart_dev.init(216);	//��ʼ��USMART
    
	IIC_Init();	
	
	Rudder_Init();        //�����ʼ�� 
	
	imu_receive_init();
	
	
	mv_init(115200);
	mvR_init(115200);
	
	QR_receive_init();
//	open_mv();
//	open_mvR();
	motor_init();
	delay_ms(2000);
	float mpuZ_reset_val;
	  //�����ǽǶȸ�λ������10��ȡƽ��ֵ
	for (uint8_t i = 0; i<10; i++)
	{
		delay_ms(20);
		mpuZ_reset_val += imu.yaw;  
	}
	mpuZ_reset_val /= 10;   // 
	mpuZreset(mpuZ_reset_val, nodesr.nowNode.angle);//�Ѵ�ʱ�Ƕȱ�Ϊ�˽��Ƕ�
}


