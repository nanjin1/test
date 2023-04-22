#ifndef __BARRIER_H
#define __BARRIER_H

#include "sys.h"
#define basic_p   5.6
#define Up_pitch   basic_p+12   //while(imu.pitch<Up_pitch)  ��ѭ�� ������
#define Down_pitch basic_p-12   //while(imu.pitch>Down_pitch)��ѭ�� ������ 
#define After_down basic_p-3    //while(imu.pitch<After_down)��ѭ������ ��ƽ��
#define After_up   basic_p+3    //while(imu.pitch>After_up)��ѭ������ ��ƽ��
extern uint8_t special_arrive;
extern uint8_t color_flag[5];
void Stage(void);
void Barrier_Bridge(float step,float speed);
void Barrier_Hill(uint8_t order) ;
void back(void);
void view1(void);//�򾰵�	
void Sword_Mountain(void);
void Barrier_HighMountain(float speed);
void Barrier_Down_HighMountain(float speed);
void view(void);
void Barrier_WavedPlate(float lenght);
void South_Pole(float L);
void QQB_1(void);
void door(void);
void route_reset(u8 flag);
void Stage_P2(void);
void S_curve(void);
void special_node(void);
void ignore_node(void);
void undermou(void);
void Special_Node(void);
void get_newroute(void);
void zhunbei(void);
#endif
