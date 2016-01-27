/*
 * Copyright (C) 2016 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

#ifndef DAC_H
#define DAC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void dac_init(void);
void dac_write(uint16_t value);

#define DAC_MAX_VALUE       0x0fff

#ifdef __cplusplus
}
#endif
#endif /* DAC_H */
