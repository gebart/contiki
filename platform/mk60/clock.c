/**
 * \file
 *         Clock module library port for MK60DZ10.
 * \author
 *         Tony Persson <tony.persson@rubico.com>
 */

#include "contiki.h"
#include "contiki-conf.h"
#include "sys/clock.h"
#include "sys/etimer.h"
#include "common.h"

#include "iroad-leds.h"
#include "stdio.h"

extern void mag_lpt_isr(void);

static volatile clock_time_t current_tick;
static volatile unsigned long current_seconds = 0;
static volatile unsigned long second_countdown = CLOCK_SECOND;

extern volatile uint8_t do_sleep;
extern volatile uint32_t ds;
/*
 * Get the system time.
 */
clock_time_t clock_time(void)
{
	return current_tick;
}

/*
 * Get the system time in seconds.
 */
unsigned long clock_seconds(void)
{
	return current_seconds;
}

/*
 * Get the system time in seconds.
 */
void clock_set_seconds(unsigned long sec)
{
	current_seconds = sec;
}

/*
 * Delay the CPU.
 */
void clock_delay(unsigned int delay)
{

}

/*
 * Delay the CPU for a number of clock ticks.
 */
void clock_wait(clock_time_t delay) 
{

}

/*
 * Initialize the clock module.
 * 
 * Generates interrupt at 1kHz.
 */
void clock_init(void)
{
    /* SIM_SCGC5: LPTIMER=1 */
    SIM_SCGC5 |= 0x01UL;

    /* Disable */
    LPTMR0_CSR = 0x00UL;

    /* LPTMR0_CSR: TCF=1,TIE=0,TPS=0,TPP=0,TFC=0,TMS=0,TEN=0 */
    LPTMR0_CSR = 0x80UL;

    /* LPTMR0_CMR: COMPARE=0x7F */
    LPTMR0_CMR = 0x7FUL; /* 64 Hz */
    //LPTMR0_CMR = 0x1FFF; /* 1 Hz */
    
    /* LPTMR0_CSR: TCF=1,TIE=1,TPS=0,TPP=0,TFC=0,TMS=0,TEN=0 */
    LPTMR0_CSR = 0xC0UL;

    /* LPTMR0_PSR: PRESCALE=1,PBYP=0,PCS=2 */
    LPTMR0_PSR = 0x0AUL;

    /* LPTMR0_CSR: TCF=0,TEN=1 */
    LPTMR0_CSR = ((LPTMR0_CSR & ~0x80UL) | 0x01UL);
    
    /* Enable LPT interrupt */
    NVICISER2 |= 0x00200000;  
}

/*
 * LPTMR ISR
 */
void lpt_isr(void)
{

    LPTMR0_CSR |= 0x80;
    do_sleep = 0;
/* iroad_leds_set(IROAD_LED3); */


#ifndef MAKE_BOOTLOADER
    /* Enable magnetometer */
    if(ds)
        mag_lpt_isr();
#endif
    /* Contiki event polling */
	current_tick++;
    
	if(etimer_pending())
	{
		etimer_request_poll();
	}

	if (--second_countdown == 0)
	{
		current_seconds++;
		second_countdown = CLOCK_SECOND;
	}
/* iroad_leds_clear(IROAD_LED3); */
}
