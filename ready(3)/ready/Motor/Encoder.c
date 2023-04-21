#include "encoder.h"
#include  <stdlib.h>
#include <string.h>
#include "uart.h"
#include "motor_task.h"
#include "speed_ctrl.h"
wheel wheel_1,wheel_2,wheel_3,wheel_4;

/*************************
	电机编码器线数13
	电机减速比144
	轮子直径104mm
*************************/
#define coefficient 0.000909090909f
#define Half_ARR 32767
uint8_t dog[4] = {0};   //电机狗

float Buffer_Encoder[4] = {0};  
float Speed[4];
//定义小车的轮子编码器属于哪个定时器
void Encoder_init(void){
	//左前
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	
	//左后
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	
	//右前
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);

	//右后
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim5);
	
	HAL_TIM_Base_Start_IT(&htim11);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM11)
	 {
	  //左前轮

	   //获取编码器计数值//>32767为正转
	   Buffer_Encoder[0]=__HAL_TIM_GetCounter(&htim1);
	   Speed[0]=(float)((Buffer_Encoder[0]-Half_ARR)* PI * coefficient);
	   __HAL_TIM_SetCounter(&htim1,32767);

	  //左后轮 
	   Buffer_Encoder[1]=__HAL_TIM_GetCounter(&htim3);
	   Speed[1]=(float)(Buffer_Encoder[1]-Half_ARR)* PI * coefficient;
	   __HAL_TIM_SetCounter(&htim3,32767); 
	  
	  //右前轮
	   Buffer_Encoder[2]=__HAL_TIM_GetCounter(&htim2);
	   Speed[2]=(float)(Buffer_Encoder[2]-Half_ARR)* PI * coefficient;
	   __HAL_TIM_SetCounter(&htim2,32767);
	  
	  //右后轮
	   Buffer_Encoder[3]=__HAL_TIM_GetCounter(&htim5);
	   Speed[3]=(float)(Buffer_Encoder[3]-Half_ARR)* PI * coefficient;
	   __HAL_TIM_SetCounter(&htim5,32767);
  
	}
	 
	if (htim->Instance == TIM14) 
	{
		HAL_IncTick();
	}
	if(htim->Instance == TIM1) 
	{
		
	}
	if(htim->Instance == TIM3) 
	{
		
	}
	if(htim->Instance == TIM2) 
	{
		
	}
	if(htim->Instance == TIM5) 
	{
		
	}

}
//重新计算脉冲
void encoder_clear(void)
{
	HAL_TIM_Encoder_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Stop(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Encoder_Stop(&htim5,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Stop(&htim5,TIM_CHANNEL_2);
	HAL_TIM_Base_Stop_IT(&htim5);
	
	motor_all.Distance = 0;
	
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim5);
	
	__HAL_TIM_SetCounter(&htim1,32767);
	__HAL_TIM_SetCounter(&htim3,32767);
	__HAL_TIM_SetCounter(&htim2,32767);
	__HAL_TIM_SetCounter(&htim5,32767);
}
