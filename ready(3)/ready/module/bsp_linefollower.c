#include "bsp_linefollower.h"
#include "usart.h"
#include "tim.h"
#include "scaner.h"
#include "stdio.h"
#include "main.h"
#include  "FreeRTOS.h"
#include  "task.h"
//此文件用于红外走
uint8_t	infrare_open=0;

volatile struct Infrared_Sensor infrared;


void get_Infrared(void){     //碰到障碍物时为1
	infrared.outside_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_10);
	infrared.inside_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
	infrared.inside_outside_right = !(uint8_t)HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8);
	
	infrared.outside_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9);
	infrared.inside_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	infrared.inside_outside_left = !(uint8_t)HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_11);
}







