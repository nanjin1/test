#include "usmart.h"
#include "usmart_str.h"
#include "pid.h"
#include "Rudder_control.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����)  	
#include "sys.h"								 
extern void led_set(u8 sta);
extern void test_fun(void(*ledset)(u8),u8 sta);										  
//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[]=
{
#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",
	(void*) usmart_pid,"void usmart_pid(uint16_t val,int deno,int mode)",
	(void*)Rudder_control,"void Rudder_control(uint16_t aim,uint8_t id)",
	(void*) chage_target,"void chage_target(uint16_t targetq)",
	(void*) speed_pid_kp,"void speed_pid_kp(int param)",
	(void*)speed_pid_kd,"void speed_pid_kd(int param)",
    (void*)	speed_pid_ki,"void speed_pid_ki(int param)",
     
	
#endif		   
//	(void*)delay_ms,"void delay_ms(u16 nms)",					
};						  
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev=
{
	usmart_nametab,
	usmart_init,
	usmart_cmd_rec,
	usmart_exe,
	usmart_scan,
	sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//��������
	0,	  	//��������
	0,	 	//����ID
	1,		//������ʾ����,0,10����;1,16����
	0,		//��������.bitx:,0,����;1,�ַ���	    
	0,	  	//ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
	0,		//�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};   



















