/*
 * Copyright (C) 2016 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     cpu_kinetis_common_pdb
 *
 * @{
 *
 * @file
 * @brief       Low-level PDB driver implementation
 *
 * This driver is a C API for controlling the PDB hardware module
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 *
 * @}
 */

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"
#include "pdb.h"

#define PDB_LDOK_BIT (BITBAND_REG32(PDB0->SC, PDB_SC_LDOK_SHIFT))

/**
 * @brief PDB trigger settings
 */
typedef enum {
    PDB_TRIG_EXTERNAL    = 0x00,
    PDB_TRIG_CMP0        = 0x01,
    PDB_TRIG_CMP1        = 0x02,
    PDB_TRIG_CMP2        = 0x03,
    PDB_TRIG_PIT_CH0     = 0x04,
    PDB_TRIG_PIT_CH1     = 0x05,
    PDB_TRIG_PIT_CH2     = 0x06,
    PDB_TRIG_PIT_CH3     = 0x07,
    PDB_TRIG_FTM0        = 0x08,
    PDB_TRIG_FTM1        = 0x09,
    PDB_TRIG_FTM2        = 0x0a,
    PDB_TRIG_RESERVED11  = 0x0b,
    PDB_TRIG_RTC_ALARM   = 0x0c,
    PDB_TRIG_RTC_SECONDS = 0x0d,
    PDB_TRIG_LPTMR       = 0x0e,
    PDB_TRIG_SOFTWARE    = 0x0f,
} pdb_trigger_t;

/**
 * @brief PDB module clock gate bitband lvalue accessor
 */
#define PDB_CLOCK_GATE (BITBAND_REG32(SIM->SCGC6, SIM_SCGC6_PDB_SHIFT))

#define PDB_CLOCK CLOCK_BUSCLOCK

static const uint32_t pdb_multipliers[] = {
    1,
    10,
    20,
    40,
};

#define PDB_MAX_MOD (PDB_MOD_MOD_MASK >> PDB_MOD_MOD_SHIFT)
#define PDB_MAX_PRESCALER (PDB_SC_PRESCALER_MASK >> PDB_SC_PRESCALER_SHIFT)

int pdb_dma_init(uint32_t freq)
{
    PDB_Type *dev = PDB0;
    PDB_CLOCK_GATE = 1;

    /* Choose the lowest possible prescaler and multiplier which can still
     * reach the target frequency to keep accuracy */
    uint32_t mult = 0;
    uint32_t prescaler = 0;
    uint32_t mod = 0;

    for (mult = 0; mult < (sizeof(pdb_multipliers) / sizeof(pdb_multipliers[0])); ++mult) {
        for (prescaler = 0; prescaler <= PDB_MAX_PRESCALER; ++prescaler) {
            uint32_t result_freq = (PDB_CLOCK / pdb_multipliers[mult]) >> prescaler;
            mod = result_freq / freq;
            if (mod <= PDB_MAX_MOD) {
                break;
            }
            mod = 0;
        }
        if (mod != 0) {
            break;
        }
    }
    if (mod == 0) {
        /* Too low frequency, unreachable with the given PDB source clock */
        return -1;
    }

    /* Turn on sequence error interrupt to catch configuration mistakes */
    /* Continuous triggers, software trigger initially, use DMA */
    dev->SC = PDB_SC_PDBEIE_MASK | PDB_SC_PDBIE_MASK |
        PDB_SC_CONT_MASK | PDB_SC_PDBEN_MASK |
        PDB_SC_TRGSEL(PDB_TRIG_SOFTWARE) | PDB_SC_DMAEN_MASK |
        PDB_SC_PRESCALER(prescaler) | PDB_SC_MULT(mult);
#if 0
    /* Enable the chosen channel and associated delay */
    dev->CH[PDB_CHANNEL].C1 =
        PDB_C1_TOS(1 << PDB_ADC_TRIGGER) | PDB_C1_EN(1 << PDB_ADC_TRIGGER);
    /* Set delay (phase) */
    dev->CH[PDB_CHANNEL].DLY[PDB_ADC_TRIGGER] = PDB_DLY_DLY(PDB_DELAY_TICKS);
#endif
    /* Set cycle length */
    dev->MOD = PDB_MOD_MOD(mod);
    /* Interrupt on each cycle start */
    dev->IDLY = PDB_IDLY_IDLY(0);
    /* Configuration done, signal PDB to load the registers */
    BITBAND_REG32(dev->SC, PDB_SC_LDOK_SHIFT) = 1;

    /* Wait until the load has been acknowledged */
    while (BITBAND_REG32(dev->SC, PDB_SC_LDOK_SHIFT)) {}

    NVIC_ClearPendingIRQ(PDB0_IRQn);
    NVIC_EnableIRQ(PDB0_IRQn);

    /* Activate software trigger */
    BITBAND_REG32(dev->SC, PDB_SC_SWTRIG_SHIFT) = 1;
    return 0;
}

void pdb_start(void)
{
    PDB_Type *dev = PDB0;
    /* Turn on continuous mode */
    BITBAND_REG32(dev->SC, PDB_SC_CONT_SHIFT) = 1;

    /* Activate software trigger */
    BITBAND_REG32(dev->SC, PDB_SC_SWTRIG_SHIFT) = 1;
}

void pdb_stop(void)
{
    PDB_Type *dev = PDB0;
    /* Turn off continuous mode */
    BITBAND_REG32(dev->SC, PDB_SC_CONT_SHIFT) = 0;
}
