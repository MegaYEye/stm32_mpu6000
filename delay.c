#include "delay.h"

static uint32_t us_ticks;
static uint32_t raw_hz;


void delay_init() {
  RCC_ClocksTypeDef clocks;

    /* compute the number of system clocks per microsecond */
    RCC_GetClocksFreq(&clocks);
    us_ticks = clocks.SYSCLK_Frequency / 1000000;
 
    raw_hz   = clocks.SYSCLK_Frequency;

    /* turn on access to the DWT registers */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* enable the CPU cycle counter */
    DWT_CTRL |= CYCCNTENA;

}
void delay_us(int us)
{
	  uint32_t elapsed    = 0;
    uint32_t last_count = DWT_CYCCNT;

    for (;;) {
        uint32_t current_count = PIOS_DELAY_GetRaw();
        uint32_t elapsed_uS;

        /* measure the time elapsed since the last time we checked */
        elapsed   += current_count - last_count;
        last_count = current_count;

        /* convert to microseconds */
        elapsed_uS = elapsed / us_ticks;
        if (elapsed_uS >= us) {
            break;
        }

        /* reduce the delay by the elapsed time */
        us -= elapsed_uS;

        /* keep fractional microseconds for the next iteration */
        elapsed %= us_ticks;
    }

    /* No error */
 
}
void delay_ms(int ms) 
{
	while (ms--) {
        delay_us(1000);
    }
 
}


