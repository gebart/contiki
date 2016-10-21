/*
 * Copyright (C) 2015 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

#ifndef MULLE_NVRAM_H_
#define MULLE_NVRAM_H_

#include "nvram.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @ingroup     board_mulle
 * @{
 *
 * @file
 * @brief       NVRAM offsets for the Eistec Mulle IoT board
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

typedef enum mulle_nvram_address {
    /** @brief NVRAM magic number, used to identify an initialized FRAM device. */
    MULLE_NVRAM_MAGIC        = 0x0000,
    /** @brief Reboot counter */
    MULLE_NVRAM_BOOT_COUNT   = 0x0004,
    /** @brief RTC time backup */
    MULLE_NVRAM_RTC_BACKUP   = 0x0008,
    /** @brief Sent packet counter */
    MULLE_NVRAM_SEND_COUNT   = 0x000C,
    /** @brief Pulse count */
    MULLE_NVRAM_PULSE_COUNT  = 0x0014,
} mulle_nvram_address_t;

#define MULLE_NVRAM_MAGIC_EXPECTED (0x4c4c554dul) /* == "MULL" in ASCII */
#define MULLE_NVRAM_CAPACITY       512     /**< FRAM size, in bytes */

extern const nvram_t *mulle_nvram;

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* MULLE_NVRAM_H_ */
