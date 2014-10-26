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
 *         SPI driver for Kinetis K60.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#ifndef CPU_ARM_K60_SPI_H_
#define CPU_ARM_K60_SPI_H_

#include <stddef.h>
#include "K60.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum spi_transfer_flag {
  SPI_TRANSFER_DONE = 0,
  SPI_TRANSFER_CONT = 1
} spi_transfer_flag_t;

typedef enum spi_transfer_sync {
  SPI_TRANSFER_ASYNC = 0,
  SPI_TRANSFER_BLOCKING = 1
} spi_transfer_sync_t;

void spi_acquire_bus(const uint8_t spi_num);
void spi_release_bus(const uint8_t spi_num);
int spi_is_busy(const uint8_t spi_num);

void spi_hw_init_master(const uint8_t spi_num);
int spi_transfer_blocking(const uint8_t spi_num, const uint8_t ctas, const uint32_t cs,
             const spi_transfer_flag_t cont, const uint8_t *data_out,
             uint8_t *data_in, size_t count_out, size_t count_in);
void spi_start(const uint8_t spi_num);
void spi_stop(const uint8_t spi_num);
void spi_set_params(const uint8_t spi_num, unsigned int ctas,
  unsigned int target_clock, unsigned int frame_size, unsigned int cpol,
  unsigned int cpha);

#define SPI_IDLE_DATA (0xffff)

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* CPU_ARM_K60_SPI_H_ */
