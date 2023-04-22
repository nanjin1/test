#include "openmv.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "uart.h"
UART_HandleTypeDef mv;//UART句柄
UART_HandleTypeDef mv_R;//UART句柄
uint8_t color;
uint8_t color_R;
void mv_init(uint32_t bound)
{	
	//UART 初始化设置
	mv.Instance=UART4;					  
	mv.Init.BaudRate=bound;				    //波特率
	mv.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	mv.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	mv.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	mv.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	mv.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&mv);					    //HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&mv, UART_IT_RXNE);
	close_mv();
}






void mvR_init(uint32_t bound)
{	
	//UART 初始化设置
	mv_R.Instance=USART6;					  
	mv_R.Init.BaudRate=bound;				    //波特率
	mv_R.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
	mv_R.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
	mv_R.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
	mv_R.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
	mv_R.Init.Mode=UART_MODE_TX_RX;		    //收发模式
	HAL_UART_Init(&mv_R);					    //HAL_UART_Init()会使能UART3
	__HAL_UART_ENABLE_IT(&mv_R, UART_IT_RXNE);
//	open_mvR()
	close_mvR();
}
void open_mv()
{
	uint8_t data=0x55;
	HAL_UART_Transmit(&mv,&data,1,0xffff);
}
void close_mv()
{
	color=0;
	uint8_t data=0x66;
	HAL_UART_Transmit(&mv,&data,1,0xffff);
}
void open_mvR()
{
	uint8_t data=0x55;
	HAL_UART_Transmit(&mv_R,&data,1,0xffff);
}
void close_mvR()
{
	color_R=0;
	uint8_t data=0x66;
	HAL_UART_Transmit(&mv_R,&data,1,0xffff);
}
void UART4_IRQHandler(void)
{
	static uint8_t temp=0;
	static uint8_t data[6]={0};
	static uint8_t flag=0;
	static uint8_t i=0;
	static uint8_t sum=0;
	if(__HAL_UART_GET_FLAG(&mv,UART_FLAG_RXNE)!=0){
		temp=mv.Instance->RDR;
		
		if(flag==2)
		{
			data[i++]=temp;
			if(i==5)
			{
				for(int m=0;m<5;m++)
				{
					sum+=data[m];
				}
				sum/=5;
			}
			else if(i==6)
			{
				memmove(&data[0],&data[1],sizeof(uint8_t) * 5);
				for(int m=0;m<5;m++)
				{
					sum+=data[m];
				}
				sum/=5;
				i=5;
			}
			if(sum==1)
			{
				color=1;
			}
			else if(sum==2)
			{
				color=2;
			}
			else if(sum==3)
			{
				color=3;
			}
			else{
				color=0;
			}
			flag=0;
		}
		if(temp==0xff&flag==1)
		{
			flag=2;
		}
		if(temp==0xff&flag==0)
		{
			flag=1;
		}
	}
	mv.Instance->ISR = 0;   //清除SR标志位
	HAL_UART_IRQHandler(&huart4);
}
void USART6_IRQHandler(void)
{
	static int temp=0;
	static uint8_t flag=0;
	if(__HAL_UART_GET_FLAG(&mv_R,UART_FLAG_RXNE)!=0){
		temp=mv_R.Instance->RDR;
		
		if(flag==2)
		{
			color_R=temp;
			flag=0;	
		}
		if(temp==0xff&flag==1)
		{
			flag=2;
		}
		if(temp==0xff&flag==0)
		{
			flag=1;
		}
	}
	mv_R.Instance->ISR = 0;   //清除SR标志位
	HAL_UART_IRQHandler(&mv_R);
}
/*****************************************************************************
函数名： bofang()
函数功能：播放歌曲
形参：无
*******************************************************************************/
void bofang()
{
	  uint8_t data[5]={0x7e,0x03,0x01,0x02,0xef};
	  HAL_UART_Transmit(&huart8,data,5,0xFFFF);
}
/*****************************************************************************
函数名： stop_bofang()
函数功能：停止播放歌曲
形参：无
*******************************************************************************/
void stop_bofang()
{
	uint8_t data[5]={0x7e,0x03,0x02,0x01,0xef};
	HAL_UART_Transmit(&huart8,data,5,0xFFFF);
}
//播放指定歌曲
void bofang_zhiding(int shou)
{
	//7E 05 41 00(歌曲高位) 01(歌曲低位) 45(校验和) EF
	  uint8_t data[7]={0x7e,0x05,0x41,0x00,0x00,0x00,0xef};
	  data[4]=shou;
	  uint8_t sum=data[1]^data[2]^data[3]^data[4];
	  data[5]=sum;
//	   HAL_UART_Transmit(&huart3,data,7,0xFFFF);
//	   while(__HAL_UART_GET_FLAG(&huart3,USART_ISR_TXE)==0)
//	   {
//		   vTaskDelay(2);
//	   }
	  for(uint8_t i=0;i<7;i++)
	  {
		  HAL_UART_Transmit(&huart8,&data[i],1,0xFFFF);
		}

}
