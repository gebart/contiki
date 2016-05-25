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
 *         Helper functions for reading Mulle platform board voltages.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#include "voltage.h"
#include "periph/adc.h"
#include "K60.h"
#include "config-board.h"

uint32_t voltage_vref_raw = 0;
uint32_t voltage_vref_millivolts = 0;

/**
 * @brief Compute VREFH voltage by measuring the 1 V band gap voltage
 */
static void _update_vref(void)
{
  /* Turn on 1 V band gap reference */
  BITBAND_REG8(PMC->REGSC, PMC_REGSC_BGBE_SHIFT) = 1;

  /* empiric evidence suggests that the band gap voltage target is not reached
   * for a long time, we busy wait for that time to avoid going to power save
   * modes as well. */
  static unsigned int delay = 20000000;
  for (unsigned int i = 0; i < delay; i++) {
    asm volatile ("nop\n");
  }

  /* read the raw value (0..65535) */
  voltage_vref_raw = adc_sample(MULLE_ADC_LINE_BANDGAP, ADC_RES_16BIT);

  /* Turn off 1 V band gap reference to save power */
  BITBAND_REG8(PMC->REGSC, PMC_REGSC_BGBE_SHIFT) = 0;

  voltage_vref_millivolts = ((MULLE_BAND_GAP_MILLIVOLTS << 16) + (voltage_vref_raw / 2)) / voltage_vref_raw;
}

void
voltage_init(void)
{
  adc_init(MULLE_ADC_LINE_BANDGAP);
  _update_vref();
  adc_init(MULLE_ADC_LINE_VCHR);
  adc_init(MULLE_ADC_LINE_VBAT);
}
uint32_t
voltage_from_raw_adc(uint32_t adc_raw)
{
  uint32_t millivolts;
  millivolts = (adc_raw * voltage_vref_millivolts) >> 16;

  return millivolts;
}
/** \todo Use interrupts to handle AD conversions of Vbat/Vchr */
uint32_t
voltage_read_vbat(void)
{
  uint32_t raw;
  uint32_t millivolts;

  /* read the raw value (0..65535) */
  raw = adc_sample(MULLE_ADC_LINE_VBAT, ADC_RES_16BIT);
  /* The ADC inputs on Vbat and Vchr are connected to a voltage divider in order
   * to be able to measure voltages greater than AVDD */
  millivolts = voltage_from_raw_adc(raw * 2);
  return millivolts;
}
uint32_t
voltage_read_vchr(void)
{
  uint32_t raw;
  uint32_t millivolts;

  /* read the raw value (0..65535) */
  raw = adc_sample(MULLE_ADC_LINE_VCHR, ADC_RES_16BIT);
  /* The ADC inputs on Vbat and Vchr are connected to a voltage divider in order
   * to be able to measure voltages greater than AVDD */
  millivolts = voltage_from_raw_adc(raw * 2);

  return millivolts;
}

uint32_t
voltage_read_avdd(void)
{
  return voltage_vref_millivolts;
}
