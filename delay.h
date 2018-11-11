#include "stm32f10x.h"
#ifndef __DELAY_H
#define __DELAY_H

#define DWT_CTRL   (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA  (1 << 0)
#define DWT_CYCCNT (*(volatile uint32_t *)0xe0001004)
#define PIOS_DELAY_GetRaw() (DWT_CYCCNT)


void delay_init(void);
void delay_us(int us);
void delay_ms(int ms);
#endif
