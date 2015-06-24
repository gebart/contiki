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
 */

#include "rtimer-arch.h"
#include "K60.h"
#include "interrupt.h"

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

/* The maximum number of ticks in the LPTMR */
#define LPTIMER_MAXTICKS (LPTMR_CNR_COUNTER_MASK >> LPTMR_CNR_COUNTER_SHIFT)

/* Any scheduling less than this many ticks into the future will cause the code
 * do a spin lock instead of setting the LPTMR compare register. */
/*  - One tick is lost when the counter is reset, because of the hardware.
 *  - One tick is wasted while waiting for synchronization.
 *  - The interrupt flag is asserted when the CNR register value matches the CMR
 *    register value and the LPTMR tick arrives, this means that CMR = 0 causes
 *    the timer to run for exactly one tick before firing. Add one for this
 *    behaviour.
 *  - Finally, add one to account for the time otherwise spent in the
 *    rtimer_schedule function, which may cause the timer to increment one more
 *    time before the synchronization increment. */
/* at 32768 Hz (LPTMR maximum), 1 tick == 30.51 us */
/* at 4096 Hz (msp430 default), 1 tick == 244.1 us */
#define RTIMER_SPINLOCK_MARGIN (4)

/* Convenience macro to get a reference to the TCF flag in the status register */
#define LPTIMER_TCF (BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TCF_SHIFT))

#ifndef LPTIMER_CNR_NEEDS_LATCHING
#warning LPTIMER_CNR_NEEDS_LATCHING is not defined in cpu_conf.h! Defaulting to 1
#define LPTIMER_CNR_NEEDS_LATCHING 1
#endif

/** Offset between current counter/timer and t=0 (boot instant) */
static volatile rtimer_clock_t offset;
static volatile rtimer_clock_t target;

#if LPTIMER_CNR_NEEDS_LATCHING

/* read the CNR register */
inline static uint32_t lptmr_get_cnr(void)
{
    uint32_t cnr;
    /* Write some garbage to CNR to latch the current value */
    asm volatile (
        /* write garbage to CNR to latch the value, it does not matter what the
         * value of r0 is. Testing shows that the write must be 32 bit wide or
         * we will get garbage when reading back CNR. */
        "str %[CNR], [%[CNR_p], #0]\n"
        "ldr %[CNR], [%[CNR_p], #0]\n" /* Read CNR */
        : /* Output variables */
        [CNR] "=&r" (cnr)
        : /* Input variables */
        [CNR_p] "r" (&(LPTMR0->CNR))
        : /* Clobbered registers */
        );
    return cnr;
}

#else /* LPTIMER_CNR_NEEDS_LATCHING */

/* read the CNR register */
inline static uint32_t lptmr_get_cnr(void)
{
    /* The early revisions of the Kinetis CPUs do not need latching of the CNR
     * register. However, this may lead to corrupt values, we read it twice to
     * ensure that we got a valid value */
    int i;
    uint32_t tmp;
    uint32_t cnr = LPTMR0->CNR;

    /* you get three retries */
    for (i = 0; i < 3; ++i) {
        tmp = LPTMR0->CNR;

        if (tmp == cnr) {
            return cnr;
        }

        cnr = tmp;
    }
    return cnr;
}

#endif /* LPTIMER_CNR_NEEDS_LATCHING */

inline static uint32_t lptmr_update_cmr(uint32_t cmr)
{
  /* The reference manual states that the CMR register should not be written
   * while the timer is enabled unless the TCF (interrupt flag) is set. */
  /* It seems like modifying the CMR variable without stopping (against the
   * reference manual's recommendations) cause sporadic failures of the
   * interrupt to trigger. It seems to happen at random. */
  /* The downside is that the CNR register is reset when the LPTMR is disabled,
   * we need a new offset computation. */
  /* Wait for the LPTMR tick counter to increase before trying to stop and
   * start the timer in order to avoid, as far as possible, losing any ticks. */
  uint32_t prev = lptmr_get_cnr();
  uint32_t now = lptmr_get_cnr();
  while (prev == now) {
    prev = now;
    now = lptmr_get_cnr();
  }

  /* Disable the timer and set the new CMR */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TEN_SHIFT) = 0;
  LPTMR0->CMR = cmr;
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TEN_SHIFT) = 1;
  return now;
}

/*
 * Initialize the clock module.
 *
 * Generates interrupt from external 32kHz crystal.
 */
/* TODO(Henrik) move to platform, init may differ between platforms. */
void
rtimer_arch_init(void) {
  offset = 0;
  target = 0;
  /* Setup Low Power Timer (LPT) */

  /* Enable LPT clock gate */
  BITBAND_REG(SIM->SCGC5, SIM_SCGC5_LPTIMER_SHIFT) = 1;

  /* Disable timer to change settings. */
  /* Logic is reset when the timer is disabled, TCF flag is also cleared on disable. */
  LPTMR0->CSR = 0x00;
  /* Prescaler bypass, LPTMR is clocked directly by ERCLK32K. */
  LPTMR0->PSR = (LPTMR_PSR_PBYP_MASK | LPTMR_PSR_PCS(0b10));
  LPTMR0->CMR = LPTMR_CMR_COMPARE(LPTIMER_MAXTICKS);
  /* Enable interrupts. */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TIE_SHIFT) = 1;
  /* Clear timer compare flag by writing a 1 to it */
  LPTIMER_TCF = 1;
  /* Enable timer */
  BITBAND_REG(LPTMR0->CSR, LPTMR_CSR_TEN_SHIFT) = 1;

  /* Enable LPT interrupt */
  NVIC_ClearPendingIRQ(LPTimer_IRQn);
  NVIC_EnableIRQ(LPTimer_IRQn);

  PRINTF("rtimer_arch_init: Done\n");
}

void
rtimer_arch_schedule(rtimer_clock_t t) {
  rtimer_clock_t now = rtimer_arch_now();
  /* there seems to be problems with the below method of running
   * rtimer_run_next() from inside rtimer_arch_schedule. It fails to reschedule
   * the clock tick rt_do_clock(). */
#if 0
  if (t < (now + RTIMER_SPINLOCK_MARGIN)) {
    /* Set target to zero to prevent ISR from interfering */
    target = 0;
    /* Spin until target is reached */
    while (t < now) {
      now = rtimer_arch_now();
    }
    rtimer_run_next();
    return;
  }
#else /* 0 */
  if (t < (now + RTIMER_SPINLOCK_MARGIN)) {
    /* delay the timer */
    t = now + RTIMER_SPINLOCK_MARGIN;
  }
#endif /* 0 */
  MK60_ENTER_CRITICAL_REGION();
  target = t; /* Update stored target time, read from ISR */
  /* Compute relative target time */
  t = t - now;
  /* The reset in lptmr_update_cmr below happens synchronously and takes
   * one LPTMR clock cycle, and the synchronization before the reset costs one
   * tick. Add the lost ticks to the counter to compensate. */
  /* Subtract one more to account for the one tick delay in asserting the
   * interrupt flag after CMR and CNR match */
  t = t - 2 - 1;
  if (t > LPTIMER_MAXTICKS) {
    /* We can not reach this time in one period, wrap around */
    t = LPTIMER_MAXTICKS;
  }
  else if (t == 0) {
    /* The synchronization loop in the next scheduling will not work if the CMR
     * is set to zero */
    t = 1;
  }
  uint32_t cnr = lptmr_update_cmr(t);
  /* Update the offset with the old counter value */
  offset += cnr;
  offset += 2;
  /* The final +1 of the offset is performed in the ISR */
  MK60_LEAVE_CRITICAL_REGION();
}

rtimer_clock_t
rtimer_arch_now(void) {
  return offset + lptmr_get_cnr();
}

/* Interrupt handler for rtimer triggers */
void
_isr_lpt(void)
{
  /* The timer counter is reset to 0 when the compare value is hit, add the
   * compare value to the 32 bit offset. Add 1 for counting the 0'th tick as
   * well (TCF asserts after CNR equals CMR and the counter increments) */
  offset += LPTMR0->CMR + 1;

  rtimer_clock_t now = rtimer_arch_now();

  if (target > now) {
    /* Target time lies in the future, we rolled over and must update
     * the compare register */
    uint32_t diff = target - offset;
    if (diff > LPTIMER_MAXTICKS) {
      diff = LPTIMER_MAXTICKS;
    }
    /* This is OK because TCF is asserted while we write CMR. */
    LPTMR0->CMR = diff;
    /* Clear timer compare flag by writing a 1 to it */
    LPTIMER_TCF = 1;
  }
  else if (target == 0) {
    /* standby run to keep rtimer_arch_now incrementing */
    LPTMR0->CMR = LPTIMER_MAXTICKS;
    /* Clear timer compare flag by writing a 1 to it */
    LPTIMER_TCF = 1;
  }
  else {
    /* Disable further rtimer_run_next calls until something was scheduled */
    target = 0;
    /* Clear timer compare flag by writing a 1 to it */
    LPTIMER_TCF = 1;
    rtimer_run_next();
  }
}
