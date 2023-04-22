#include "QR.h"
#include "map.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
uint8_t BW_add=0;
#define QRBUFFER_SIZE 8
uint8_t QR_rx_buf[QRBUFFER_SIZE] = {0};
uint8_t QR_rx_len = 0;

void QR_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart7,QR_rx_buf,QRBUFFER_SIZE);
}
void UART7_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	flag_idle = __HAL_UART_GET_FLAG(&huart7,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart7);
		HAL_UART_DMAStop(&huart7); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_uart7_rx); 		
		QR_rx_len = QRBUFFER_SIZE - temp; 
		if(QR_rx_buf[0] == 0x48&QR_rx_buf[1]==0X45&QR_rx_buf[2]==0X41&QR_rx_buf[3]==0X44)
		{
			switch(QR_rx_buf[4])
			{
					case 0x33:
						BW_add=3;
					    break;
					case 0x34:
						BW_add=4;
						break;
					case 0x35:
						BW_add=5;
						break;
					case 0x36:
						BW_add=6;
						break;
					case 0x37:
						BW_add=7;
						break;
					case 0x38:
						BW_add=8;
						break;
					default:
						break;
				}
		   }
		memset(QR_rx_buf,0,QR_rx_len);
		QR_rx_len = 0;
	}
	HAL_UART_Receive_DMA(&huart7,QR_rx_buf,QRBUFFER_SIZE);
	HAL_UART_IRQHandler(&huart7);
}

//// 前缀为十六进制48 45 41 44
//void USART2_IRQHandler(void)
//{
//	static uint8_t temp=0;
//	static uint8_t data[5]={0};
//	static uint8_t i=0;
//	if(__HAL_UART_GET_FLAG(&QR,UART_FLAG_RXNE)!=0){
//		temp=QR.Instance->RDR;
//		data[i]=temp;
//		i++;
//		if(i==5)
//		{
//			if(data[0]==0x48&data[1]==0x45&data[2]==0x41&data[3]==0x44)
//			{
//				switch(data[4])
//				{
//					case 0x33:
//						BW_add=3;
//					    break;
//					case 0x34:
//						BW_add=4;
//						break;
//					case 0x35:
//						BW_add=5;
//						break;
//					case 0x36:
//						BW_add=6;
//						break;
//					case 0x37:
//						BW_add=7;
//						break;
//					case 0x38:
//						BW_add=8;
//						break;
//					default:
//						break;
//				}
//			}
//			i=0;
//		}
//	}
//	QR.Instance->ISR = 0;   //清除SR标志位
//}
int BW_num[3]={0,0,0};
int check_BW(u16 node)
{
	switch(node)
	{
		case P3:
			if(BW_num[0] == 3) return 1;else return 0;//3   
		case P4:
			if(BW_num[0] == 4) return 1;else return 0;//4
		case P5:
			if(BW_num[1] == 6) return 1;else return 0;//6
		case P6:
			if(BW_num[1] == 5) return 1;else return 0;//5
		case P7:
			if(BW_num[2] == 8) return 1;else return 0;//8
		case P8:
			if(BW_num[2] == 7) return 1;else return 0;//7
		default: return 0;
	}
		
}
//二维码1-31 2-32  以此类推
 
