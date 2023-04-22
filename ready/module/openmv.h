#ifndef __openmv_h__
#define __openmv_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
extern uint8_t color;
extern uint8_t color_R;
void open_mv(void);
void close_mv(void);
void open_mvR(void);
void close_mvR(void);
void mv_init(uint32_t bound);
void mvR_init(uint32_t bound);
void bofang_zhiding(int shou);
void stop_bofang(void);
void bofang(void);
extern UART_HandleTypeDef mv_R;//UART¾ä±ú
#endif
