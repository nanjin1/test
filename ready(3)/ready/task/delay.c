#include "delay.h"
#include "encoder.h"
#include "motor.h"
#include "temporary_task.h"
#include "bsp_led.h"
#include "tim.h"
#define detect_dog 3
//��ʱ����+�����
TIM_HandleTypeDef TIM7_Handler;
void delay_init(void)
{
    //��ʱ��7
    __HAL_RCC_TIM7_CLK_ENABLE();
     
    TIM7_Handler.Instance=TIM7;                          //ͨ�ö�ʱ��7
    TIM7_Handler.Init.Prescaler=216-1;                     //��Ƶ
    TIM7_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM7_Handler.Init.Period=1500-1;                        //�Զ�װ��ֵ
    TIM7_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	TIM7_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&TIM7_Handler);
	HAL_NVIC_SetPriority(TIM7_IRQn,9,0);    //�����ж����ȼ�����ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM7_IRQn);          //����ITM4�ж�  
    HAL_TIM_Base_Start_IT(&TIM7_Handler); //ʹ�ܶ�ʱ��7�Ͷ�ʱ��7�ж� 
}

//1.5ms  
void TIM7_IRQHandler(void)
{
	static uint8_t mouse = 0;    //С����
    if(__HAL_TIM_GET_IT_SOURCE(&TIM7_Handler,TIM_IT_UPDATE)==SET)//����ж�
    {
//					for(uint8_t i=0;i<4;i++){
//						dog[i]++;   //�����
////						if(dog[i]>=detect_dog)
////							Speed[i]=0;
//						if(dog[i]>=255){
//									dog[i]=0;   //�����
//						}
										
					mouse++;
					if(mouse>100){
						mouse =0;
//						LED_twinkle();
						HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
					}
		}
    __HAL_TIM_CLEAR_IT(&TIM7_Handler, TIM_IT_UPDATE);//����жϱ�־λ
	HAL_TIM_IRQHandler(&TIM7_Handler);
}

//�����ʱ1500us
void delay_us(uint16_t nus)
{
   TIM7->CNT = 0;
	 while(TIM7->CNT<nus);
}

void delay_ms(uint16_t nms)
{
	for(uint16_t i=0;i<nms;i++){
		delay_us(1000);
	}
}
