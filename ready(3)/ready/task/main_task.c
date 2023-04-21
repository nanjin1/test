#include "main_task.h"
#include "rudder_control.h"
#include "uart.h"
#include "imu_task.h"
#include "uart.h"
#include "turn.h"
#include "map.h"
#include "barrier.h"
#include "bsp_buzzer.h"
#include "bsp_linefollower.h"
#include "scaner.h"
#include "speed_ctrl.h"
#include "encoder.h"
#include "barrier.h"
#include "motor_task.h"
#include "openmv.h"
#include "math.h"
#include "barrier.h"
void main_task(void *pvParameters){
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();   //��ȡϵͳ����
zhunbei();


 while(1){
	if(map.routetime==1)
	 { 
		map.routetime=2;
		get_newroute();
		float mpuZ_reset_val;
	    for (uint8_t i = 0; i<10; i++)
	    {
		   delay_ms(20);
		   mpuZ_reset_val += imu.yaw;  
	    }
	   mpuZ_reset_val /= 10;   // 
	   mpuZreset(mpuZ_reset_val, nodesr.nowNode.angle);//�Ѵ�ʱ�Ƕȱ�Ϊ�˽��Ƕ�
		zhunbei(); 
	 }
	  Cross();
	 if(map.routetime==3)
	 {
		 CarBrake();
		 while(1)
		 {
			 vTaskDelay(2);
		 }
	 }

	vTaskDelayUntil(&xLastWakeTime, (5/portTICK_RATE_MS));//��������5ms // INCLUDE_vTaskDelayUntil 1

	}
}
