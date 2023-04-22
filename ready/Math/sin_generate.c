#include "sin_generate.h"
#include "math.h"

#define PI 3.1415926535f
TIM_HandleTypeDef TIM6_Handler;      //��ʱ����� 
struct sin_param sin1={0,0,1000,0.15};
float sin_generator(struct sin_param *param)
{
	float output;
	
	param->actual_t = param->time * param->angular_velocity;
	
	output = param->gain * sin(param->actual_t * PI/180.0f);
	
	++param->time;
	
	if (param->actual_t >= 360)
		param->time = 0;
	
	return output;
}
void Timer6_Init()
{	//��ʱ��5 1ms
   __HAL_RCC_TIM6_CLK_ENABLE();
     
    TIM6_Handler.Instance=TIM6;                          //ͨ�ö�ʱ��4
    TIM6_Handler.Init.Prescaler=215;                     //��Ƶ
    TIM6_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM6_Handler.Init.Period=999;                        //�Զ�װ��ֵ
    TIM6_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&TIM6_Handler);
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn,0,0);    //�����ж����ȼ�����ռ���ȼ�3�������ȼ�3
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          //����ITM4�ж�  
    HAL_TIM_Base_Start_IT(&TIM6_Handler); //ʹ�ܶ�ʱ��4�Ͷ�ʱ��4�ж� 
}

void TIM6_DAC_IRQHandler(void)
{ 	
    if(__HAL_TIM_GET_IT_SOURCE(&TIM6_Handler,TIM_IT_UPDATE)==SET)//����ж�
    {
		sin1.time++;
    }
    __HAL_TIM_CLEAR_IT(&TIM6_Handler, TIM_IT_UPDATE);//����жϱ�־λ
}
