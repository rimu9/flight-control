#ifndef _SYS_FUN_H_
#define _SYS_FUN_H_

#include "stm32f10x.h"

void NVIC_INIT(void);
char SystemClock_HSI(u8 PLL);
char SystemClock_HSE(u8 PLL);

extern char SysClock;       

#endif
