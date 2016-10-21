/**
 * \file
 *         Provide udelay routine for MK60DZ10.
 * \author
 *         Tony Persson <tony.persson@rubico.com>
 */

#include "K60.h"
#include "config-clocks.h"
#include "power-modes.h"
#include "irq.h"

/**
 * @brief spin instead of waiting if the number of ticks is less than this number
 */
#define UDELAY_SPIN_LIMIT 1000

void
udelay_init(void)
{
  /* Enable peripheral clock */
  SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;
  /* Reset logic to a known state */
  PIT->MCR = (PIT_MCR_FRZ_MASK);
  NVIC_ClearPendingIRQ(PIT3_IRQn);
}

/**
 * Microsecond blocking wait.
 *
 * \param [in] us number of microseconds to sleep.
 */
void
udelay(unsigned int us)
{
  /* Don't hang on zero µs sleep. */
  if (us == 0) return;

  uint32_t timeout = ((F_BUS) / 1000000) * us;
  if (timeout < us) {
    /* Timeout is too large to fit in counter, the udelay routine should not be
     * used for these kinds of waits. */
    timeout = ~0;
  }

  /* Set up timer */
  PIT->CHANNEL[3].LDVAL = PIT_LDVAL_TSV(timeout);

  /* Disable timer to load a new value */
  PIT->CHANNEL[3].TCTRL = 0;

  /* Clear interrupt flag */
  PIT->CHANNEL[3].TFLG = PIT_TFLG_TIF_MASK;

  if (timeout < UDELAY_SPIN_LIMIT) {
    /* spin instead of relying on IRQ+wait */
    PIT->CHANNEL[3].TCTRL = PIT_TCTRL_TEN_MASK;
    while(!(PIT->CHANNEL[3].TFLG & PIT_TFLG_TIF_MASK)) {}
  }
  else {
    /* Enable timer, interrupt, no chaining */
    PIT->CHANNEL[3].TCTRL = (PIT_TCTRL_TEN_MASK | PIT_TCTRL_TIE_MASK);

    /* Wait for completion, IRQs which occur with IRQs masked will still kick the
     * CPU from low power mode */
    NVIC_EnableIRQ(PIT3_IRQn);
    while(!(PIT->CHANNEL[3].TFLG & PIT_TFLG_TIF_MASK)) {
      power_mode_wait();
    }
    NVIC_DisableIRQ(PIT3_IRQn);
  }

  /* Disable everything */
  PIT->CHANNEL[3].TCTRL = 0;
}

void isr_pit3(void)
{
  /* This IRQ will wake the CPU */
  /* flags will be handled in udelay */
  NVIC_DisableIRQ(PIT3_IRQn);
  NVIC_ClearPendingIRQ(PIT3_IRQn);
  /* Disable timer */
  PIT->CHANNEL[3].TCTRL = 0;
  //~ volatile uint32_t tflg = PIT->CHANNEL[3].TFLG;
  //~ (void) tflg;
}
