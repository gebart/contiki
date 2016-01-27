/*
 * Copyright (C) 2016 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kinetis_common_dac
 *
 * @{
 *
 * @file
 * @brief       Low-level DAC driver implementation
 *
 * This driver uses the 12-bit in-CPU DAC module. The 6-bit digital-to-analog
 * converter inside the CMP module is _not_ used by this implementation.
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */

#include <stdint.h>

#include "K60.h"
#include "dac.h"

/* DAC 0 configuration */
#define DAC_0_DEV           DAC0
#define DAC_0_CLKEN()       (BITBAND_REG32(SIM->SCGC2, SIM_SCGC2_DAC0_SHIFT) = 1)
#define DAC_0_CLKDIS()      (BITBAND_REG32(SIM->SCGC2, SIM_SCGC2_DAC0_SHIFT) = 0)

void dac_init(void)
{
    DAC_Type *dac;
    dac = DAC_0_DEV;
    /* Activate clock gate */
    DAC_0_CLKEN();

    /* Select VDDA as voltage reference */
    dac->C0 = (DAC_C0_DACRFS_MASK);

    /* Disable DMA and buffering */
    dac->C1 = 0;
    dac->C2 = 0;

    /* Enable DAC */
    BITBAND_REG32(dac->C0, DAC_C0_DACEN_SHIFT) = 1;

    /* Set output value to zero */
    dac_write(0);
}

void dac_write(uint16_t value)
{
    DAC_Type *dac;
    dac = DAC_0_DEV;

    if (value > DAC_MAX_VALUE) {
        value = DAC_MAX_VALUE;
    }

    dac->DAT[0].DATH = ((value >> 8) & 0xff);
    dac->DAT[0].DATL = (value & 0xff);
}
