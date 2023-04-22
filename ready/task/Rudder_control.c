#include "Rudder_control.h"
#include "uart.h"
//#include  "uart.h"
// iic
void Rudder_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOC_CLK_ENABLE();
	IIC_Init();//IIC��ʼ��
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);//OEʹ��
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	Rudder_WriteOneByte(0x0,0x1);//��Ӧ0x70ͨ��iic��ַ���ҹر�sleepģʽ
//	����Ƶ��
	uint8_t /*prescale,*/oldmode,newmode;
	oldmode = Rudder_ReadOneByte(0x0);
	newmode = (oldmode&0x7F) | 0x10; // sleep
	Rudder_WriteOneByte(0x0, newmode); // go to sleep
    Rudder_WriteOneByte(PRE_SCALE, 132); // set the prescaler
	Rudder_WriteOneByte(0x0,0x1);//��Ӧ0x70ͨ��iic��ַ���ҹر�sleepģʽ
	
//	int temp =1;
//	temp = Rudder_ReadOneByte(PRE_SCALE);
//	printf("%d\r\n",temp);
}

//��ָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ    
//DataToWrite:Ҫд�������
void Rudder_WriteOneByte(uint8_t WriteAddr,uint8_t DataToWrite)
{
	IIC_Start();  	
	IIC_Send_Byte(Rudder<<1);    //����������ַ0x55,д���� 	 
	IIC_Wait_Ack();
    IIC_Send_Byte(WriteAddr);   //����Ŀ���ַ
	IIC_Wait_Ack();	
	IIC_Send_Byte(DataToWrite);//�����ֽ�							   
	IIC_Wait_Ack();  		    	   
    IIC_Stop();//����һ��ֹͣ���� 
	delay_ms(10);	 
}
//ָ����ַ����һ������
//ReadAddr:��ʼ�����ĵ�ַ  
//����ֵ  :����������
uint8_t Rudder_ReadOneByte(uint8_t ReadAddr)
{				  
	uint8_t temp=0;		  	    																 
	IIC_Start();  
	IIC_Send_Byte(Rudder<<1);   //����������ַ0X70,д���� 	   
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);   //����Ŀ���ַ
	IIC_Wait_Ack();	
	IIC_Start();  	 	   
	IIC_Send_Byte((Rudder<<1)+1);           //�������ģʽ			   
	IIC_Wait_Ack();	 
    temp=IIC_Read_Byte(0);		   
    IIC_Stop();					//����һ��ֹͣ����	    
	return temp;
}
/*****************************************************************************
��������  Rudder_control()
�������ܣ��������
�βΣ� aim---Ŀ��Ƕ�    ID�����id
��ע��0�ȶ�Ӧ350    270�ȶ�Ӧ2100  
*******************************************************************************/
void Rudder_control(uint16_t aim,uint8_t id)
{
		Rudder_WriteOneByte(LED0_ON_L+(4*id),0x0);
		Rudder_WriteOneByte(LED0_ON_H+(4*id),0x0);
		
		Rudder_WriteOneByte(LED0_OFF_L+(4*id),(uint8_t)(aim&0xFF));
		Rudder_WriteOneByte(LED0_OFF_H+(4*id),(uint8_t)(aim>>8));
}
