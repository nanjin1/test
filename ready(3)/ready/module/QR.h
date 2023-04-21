#ifndef __QR_h__
#define __QR_h__
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "sys.h"

extern uint8_t BW_add;
int check_BW(u16 node);
void QR_receive_init(void);
void QR_init(uint32_t bound);
#endif
