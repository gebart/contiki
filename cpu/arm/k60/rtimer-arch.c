/*
 * Copyright (c) 2014, Eistec AB.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * This file is part of the Mulle platform port of the Contiki operating system.
 *
 */

/**
 * \file
 *         K60 specific rtimer library implementation.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 *         Henrik Makitaavola <henrik@makitaavola.se>
 */

#include "rtimer-arch.h"
#include "K60.h"
#include "clock.h"
#include "irq.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define LPTMR_MAX_VALUE (LPTMR_CNR_COUNTER_MASK >> LPTMR_CNR_COUNTER_SHIFT)

/* Write some garbage to CNR to latch the current value */
/* A bug in the MK60D10.h header causes errors since the CNR field is const
 * declared, cast to volatile uint32_t* as a workaround. */
#define LPTMR0_LATCH_CNR() (*((volatile uint32_t*)(&(LPTMR0->CNR))) = 0)

/* The period length of the rtimer, in rtimer ticks */
#define RTIMER_PERIOD ((LPTMR_CNR_COUNTER_MASK >> LPTMR_CNR_COUNTER_SHIFT) + 1)

/* Number of ticks into the future for us to not consider the rtimer as immediately expired */
/* at 32768 Hz (LPTMR maximum), 1 tick == 30.51 us */
/* at 4096 Hz (msp430 default), 1 tick == 244.1 us */
#define RTIMER_SCHEDULE_MARGIN (2)


#ifndef RTC_LOAD_CAP_BITS
#define RTC_LOAD_CAP_BITS    0
#endif

/*
 * The RTC prescaler will normally count to 32767 every second unless configured
 * otherwise through the time compensation register.
 */
#define TIMER_RTC_SUBTICK_MAX (0x7fff)
/*
 * Number of bits in the ideal RTC prescaler counter
 */
#define TIMER_RTC_SUBTICK_BITS (15)

/**
 * @brief  The number of ticks that will be lost when setting a new target in the LPTMR
 *
 * The counter will otherwise drop ticks when setting new timeouts.
 */
#define LPTMR_RELOAD_OVERHEAD 2

/*----------------------------------------------------------------------------*/
inline static rtimer_clock_t _rtt_get_subtick(void)
{
  volatile uint32_t tpr;
  volatile uint32_t tsr;

  unsigned int mask = irq_disable();
  for (int i = 0; i < 5; i++) {
    /* Read twice to make sure we get a stable reading */
    tpr = RTC->TPR & RTC_TPR_TPR_MASK;
    tsr = RTC->TSR & RTC_TSR_TSR_MASK;

    if ((tsr == (RTC->TSR & RTC_TSR_TSR_MASK)) &&
        (tpr == (RTC->TPR & RTC_TPR_TPR_MASK))) {
      break;
    }
  }
  if (tpr > TIMER_RTC_SUBTICK_MAX) {
    /* This only happens if the RTC time compensation value has been
     * modified to compensate for RTC drift. See Kinetis ref.manual,
     *  RTC Time Compensation Register (RTC_TCR).
     */
    tpr = TIMER_RTC_SUBTICK_MAX;
  }

  irq_restore(mask);
  return (tsr << TIMER_RTC_SUBTICK_BITS) | tpr;
}
/*----------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  PRINTF("RTIMTER_ARCH: Init\n");

  /* ------ Setup RTC ------ */
  RTC->CR = RTC_CR_SWR_MASK;
  RTC->CR = 0;

  if (RTC->SR & RTC_SR_TIF_MASK) {
    /* Clear TIF by writing TSR. */
    RTC->TSR = 0;
  }

  /* Enable RTC oscillator and non-supervisor mode accesses. */
  /* Enable load capacitance as configured by periph_conf.h */
  RTC->CR = RTC_CR_OSCE_MASK | RTC_CR_SUP_MASK | RTC_LOAD_CAP_BITS;

  /* Clear TAF by writing TAR. */
  RTC->TAR = 0xffffff42;

  /* Disable all RTC interrupts. */
  RTC->IER = 0;
  RTC->SR |= RTC_SR_TCE_MASK;

  /* ------ Setup Low Power Timer (LPT) ------*/
  /* Enable LPT clock gate */
  BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_LPTIMER_SHIFT) = 1;
  /* Completely disable the module before messing with the settings */
  LPTMR0->CSR = 0x00;
  /* select ERCLK32K as clock source for LPTMR */
  LPTMR0->PSR = (LPTMR_PSR_PBYP_MASK | LPTMR_PSR_PCS(0b10));
  /* Clear IRQ flag in case it was already set */
  LPTMR0->CSR = LPTMR_CSR_TCF_MASK;
  /* Enable IRQs on the counting channel */
  /* Refactor the below lines if there are any CPUs where the LPTMR IRQs are not sequential */
  NVIC_ClearPendingIRQ(LPTMR0_IRQn);
  NVIC_EnableIRQ(LPTMR0_IRQn);
}
/*----------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  PRINTF("RTIMER_ARCH: Schedule %lu\n", t);
  /* Disable IRQs to minimize jitter */
  unsigned int mask = irq_disable();
  LPTMR0->CSR = 0;

  rtimer_clock_t now = RTIMER_NOW();

  if (t < now+RTIMER_SCHEDULE_MARGIN)
  {
    t = now+RTIMER_SCHEDULE_MARGIN;
  }
  LPTMR0->CMR = t-now+LPTMR_RELOAD_OVERHEAD;
  LPTMR0->CSR = LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK;
  irq_restore(mask);
}
/*----------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_arch_now(void)
{
  return _rtt_get_subtick();
}
/*----------------------------------------------------------------------------*/
/* Interrupt handler for rtimer triggers */
void
isr_lptmr0(void)
{
  /* Clear interrupt flag */
  BITBAND_REG32(LPTMR0->CSR, LPTMR_CSR_TCF_SHIFT) = 1;
  rtimer_run_next();
}

