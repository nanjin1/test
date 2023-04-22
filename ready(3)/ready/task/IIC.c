#include "iic.h"

void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOD_CLK_ENABLE();   //ʹ��GPIODʱ��
    
    //PD12,PD13��ʼ������
    GPIO_Initure.Pin=GPIO_PIN_11|GPIO_PIN_12;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
    
    IIC_SDA_OUT;
    IIC_SCK_OUT;  
}
void SDA(uint8_t param){
	GPIO_InitTypeDef GPIO_Initure={0};
	if(param==1){
	GPIO_Initure.Pin=GPIO_PIN_11;
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	}
	if(param==0){
	GPIO_Initure.Pin=GPIO_PIN_11;
    GPIO_Initure.Mode=GPIO_MODE_INPUT;        //����
    GPIO_Initure.Pull=GPIO_PULLDOWN;          //����
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;  //����
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	}
	
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA(0x01);     //sda�����
	IIC_SDA_OUT;	  	  
	IIC_SCK_OUT;
	delay_us(4);
 	IIC_SDA_DOWN;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCK_DOWN;//ǯסI2C���ߣ�׼�����ͻ�������� 
}
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA(0x01);     //sda�����
	IIC_SCK_DOWN;
	IIC_SDA_DOWN;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCK_OUT; 
	IIC_SDA_OUT;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA(0X00);      //SDA����Ϊ����  
	IIC_SDA_OUT;delay_us(1);	   
	IIC_SCK_OUT;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCK_DOWN;//ʱ�����0 	   
	return 0;  
}
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCK_DOWN;
	SDA(0X1);
	IIC_SDA_DOWN;
	delay_us(2);
	IIC_SCK_OUT;
	delay_us(2);
	IIC_SCK_DOWN;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCK_DOWN;
	SDA(0X01);
	IIC_SDA_OUT;
	delay_us(2);
	IIC_SCK_OUT;;
	delay_us(2);
	IIC_SCK_DOWN;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
  uint8_t t;   
  SDA(0x01); 	    
  IIC_SCK_DOWN;//����ʱ�ӿ�ʼ���ݴ���
  for(t=0;t<8;t++)
  {
		if((txd&0x80)>>7){
			IIC_SDA_OUT;
		}
		else{
			IIC_SDA_DOWN;
		}
		
    txd<<=1; 	  
	delay_us(2);   //��TEA5767��������ʱ���Ǳ����
	IIC_SCK_OUT;
	delay_us(2); 
	IIC_SCK_DOWN;	
	delay_us(2);
  }	 
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA(0x0);//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCK_DOWN; 
        delay_us(2);
		IIC_SCK_OUT;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}
