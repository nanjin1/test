#include "imu_task.h"
#include "usart.h"
#include "main.h"
#include "uart.h"
#include "stdio.h"
#include "string.h"
struct Imu imu;

UART_HandleTypeDef gyro;//UART���

void gyro_init(uint32_t bound)
{ 
 //UART ��ʼ������
 gyro.Instance=USART3;         //USART2
 gyro.Init.BaudRate=bound;        //������
 gyro.Init.WordLength=UART_WORDLENGTH_8B;   //�ֳ�Ϊ8λ���ݸ�ʽ
 gyro.Init.StopBits=UART_STOPBITS_1;     //һ��ֹͣλ
 gyro.Init.Parity=UART_PARITY_NONE;      //����żУ��λ
 gyro.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //��Ӳ������
 gyro.Init.Mode=UART_MODE_TX_RX;      //�շ�ģʽ
 HAL_UART_Init(&gyro);         //HAL_UART_Init()��ʹ��UART3
// __HAL_UART_ENABLE_IT(&gyro, UART_IT_RXNE);
}

#define BUFFER_SIZE 15
uint8_t imu_rx_buf[BUFFER_SIZE] = {0};
uint8_t imu_rx_len = 0;
float roll,pitch,yaw;

void imu_receive_init(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,imu_rx_buf,BUFFER_SIZE);
}

void USART3_IRQHandler(void)
{
	uint32_t flag_idle = 0;
	
	flag_idle = __HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); 
	if((flag_idle != RESET))
	{ 
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);

		HAL_UART_DMAStop(&huart3); 
		uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);   
		imu_rx_len = BUFFER_SIZE - temp; 
	
		if(imu_rx_buf[0] == 0x55)
		{
			uint8_t sum = 0;
			for (int i=0; i<10; i++)
				sum += imu_rx_buf[i];
			if (sum == imu_rx_buf[10])
			{
				if (imu_rx_buf[2] == 0X01)
				{
					imu.roll   = 180.0 * (short) ((imu_rx_buf[5]<<8)|imu_rx_buf[4])/32768.0;  
					imu.pitch  = 180.0 * (short) ((imu_rx_buf[7]<<8)|imu_rx_buf[6])/32768.0;//����(��Ϊ��)
					imu.yaw    = 180.0 * (short) ((imu_rx_buf[9]<<8)|imu_rx_buf[8])/32768.0;
//					printf("roll=%f,pitch=%f,yaw=%f \r\n",imu.roll,imu.pitch,imu.yaw);
				}
			}
		}
		memset(imu_rx_buf,0,imu_rx_len);
		imu_rx_len = 0;
	}
	HAL_UART_Receive_DMA(&huart3,imu_rx_buf,BUFFER_SIZE);
	HAL_UART_IRQHandler(&huart3);
}
