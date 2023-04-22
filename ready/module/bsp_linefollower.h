#ifndef __BSP_LINEFOLLOWER_H
#define __BSP_LINEFOLLOWER_H

#include "sys.h"

#define LFB_SENSOR_NUM 16   //循迹板传感器数量
#define LFB_HALF_SENSOR_NUM 8

//向前的红外
#define Infrared_ahead  (uint8_t)!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)
extern uint8_t	infrare_open;
void get_Infrared(void);

struct Infrared_Sensor    //红外传感器
{
	uint8_t outside_left;          //车头左边
	uint8_t outside_right;         //车头右边
	uint8_t inside_left;           //车底里面左边
	uint8_t inside_right;          //车底里面右边
	uint8_t inside_outside_left;   //车底外面左边 
	uint8_t inside_outside_right;  //车底外面右边
};


extern volatile struct Infrared_Sensor infrared;


void LFB_send(void);


#endif

