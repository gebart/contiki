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

#ifndef MULLE_DEV_VOLTAGE_H_
#define MULLE_DEV_VOLTAGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t voltage_vref_raw;
extern uint32_t voltage_vref_millivolts;


void voltage_init(void);
/**
 * @brief Scale a raw ADC reading from 0..65535 to millivolts depending on the board's
 * VREFH, VREFL reference voltages.
 */
uint32_t voltage_from_raw_adc(uint32_t adc_raw);
uint32_t voltage_read_vbat(void);
uint32_t voltage_read_vchr(void);
uint32_t voltage_read_avdd(void);

/**
 * @brief Convert a raw ADC reading to voltage in Qn fixed point format
 *
 * @param[in] adc_raw   raw ADC reading, left adjusted (Q0.16)
 * @param[in] frac_bits number of fractional bits in the result, n in Qx.n
 *
 * @return Voltage as a Qx.n fixed point number
 */
static inline uint32_t voltage_adc_as_fixpoint(uint16_t adc_raw, uint8_t frac_bits)
{
    /* Q/dq = A/da * dr/R, da=dr => Q = A/R * dq */
    uint32_t res = adc_raw;
    /* Scale to Qx.n (pre-multiply dq) */
    res <<= frac_bits;
    /* add rounding factor */
    res += voltage_vref_raw / 2;
    /* divide by reference measurement */
    res /= voltage_vref_raw;
    return res;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MULLE_DEV_VOLTAGE_H_ */
