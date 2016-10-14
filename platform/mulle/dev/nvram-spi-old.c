/*
 * Copyright (C) 2015 Eistec AB
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

#include <stdint.h>
#include <stddef.h>
#include "nvram.h"
#include "nvram-spi-old.h"
#include "irq.h"

/**
 * @ingroup     nvram
 * @{
 *
 * @file
 *
 * @brief       Device interface for various SPI connected NVRAM
 *
 * This implementation uses the old Eistec Contiki SPI driver API.
 *
 * Tested on:
 * - Cypress/Ramtron FM25L04B.
 *
 * @author      Joakim Nohlgård <joakim.nohlgard@eistec.se>
 */

typedef enum {
    /** WRITE command byte, 0b0000 0010 */
    NVRAM_SPI_CMD_WRITE = 0x02,
    /** READ command byte, 0b0000 0011 */
    NVRAM_SPI_CMD_READ = 0x03,
    /** WREN command byte, 0b0000 0110 */
    NVRAM_SPI_CMD_WREN = 0x06,
} nvram_spi_commands_t;

/** @brief Delay to wait between toggling CS pin, on most chips this can probably be
 * removed. */
#define NVRAM_SPI_CS_TOGGLE_TICKS 1

/**
 * @brief Copy data from system memory to NVRAM.
 *
 * This is a special form of the WRITE command used by some Ramtron/Cypress
 * 4Kbit FRAM devices which puts the 9th address bit inside the command byte to
 * be able to use one byte for addressing instead of two.
 *
 * @param[in]  dev   Pointer to NVRAM device descriptor
 * @param[in]  src   Pointer to the first byte in the system memory address space
 * @param[in]  dst   Starting address in the NVRAM device address space
 * @param[in]  len   Number of bytes to copy
 *
 * @return           Number of bytes written on success
 * @return           <0 on errors
 */
static int nvram_spi_write_9bit_addr(const nvram_t *dev, uint8_t *src, uint32_t dst, size_t len);

/**
 * @brief Copy data from NVRAM to system memory.
 *
 * This is a special form of the READ command used by some Ramtron/Cypress 4Kbit
 * FRAM devices which puts the 9th address bit inside the command byte to be
 * able to use one byte for addressing instead of two.
 *
 * @param[in]  dev   Pointer to NVRAM device descriptor
 * @param[out] dst   Pointer to the first byte in the system memory address space
 * @param[in]  src   Starting address in the NVRAM device address space
 * @param[in]  len   Number of bytes to copy
 *
 * @return           Number of bytes read on success
 * @return           <0 on errors
 */
static int nvram_spi_read_9bit_addr(const nvram_t *dev, uint8_t *dst, uint32_t src, size_t len);

int nvram_spi_init(nvram_t *dev, nvram_spi_params_t *spi_params, size_t size)
{
    dev->size = size;
    dev->write = nvram_spi_write_9bit_addr;
    dev->read = nvram_spi_read_9bit_addr;
    dev->extra = spi_params;

    return 0;
}

static int nvram_spi_write_9bit_addr(const nvram_t *dev, uint8_t *src, uint32_t dst, size_t len)
{
    nvram_spi_params_t *spi_dev = dev->extra;
    int status;
    uint8_t cmd;
    uint8_t addr;
    /* LSB of address */
    addr = (dst & 0xff);
    K60_ENTER_CRITICAL_REGION();
    spi_acquire_bus(spi_dev->spi);
    /* Enable writes */
    cmd = NVRAM_SPI_CMD_WREN;
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_DONE, &cmd, NULL, 1, 0);
    if (status < 0)
    {
        return status;
    }
    cmd = NVRAM_SPI_CMD_WRITE;
    /* The upper address bit is mixed into the command byte on certain devices,
     * probably just to save a byte in the SPI transfer protocol. */
    if (dst > 0xff) {
        cmd |= 0x08;
    }
    //xtimer_spin(NVRAM_SPI_CS_TOGGLE_TICKS);
    /* Write command and address */
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_CONT, &cmd, NULL, 1, 0);
    if (status < 0)
    {
        return status;
    }
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_CONT, &addr, NULL, 1, 0);
    if (status < 0)
    {
        return status;
    }
    /* Keep holding CS and write data */
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_DONE, src, NULL, len, 0);
    if (status < 0)
    {
        return status;
    }
    spi_release_bus(spi_dev->spi);
    K60_LEAVE_CRITICAL_REGION();
    /* status is 0 in Contiki if all went well. */
    return len;
}

static int nvram_spi_read_9bit_addr(const nvram_t *dev, uint8_t *dst, uint32_t src, size_t len)
{
    nvram_spi_params_t *spi_dev = dev->extra;
    int status;
    uint8_t cmd;
    uint8_t addr;
    cmd = NVRAM_SPI_CMD_READ;
    /* The upper address bit is mixed into the command byte on certain devices,
     * probably just to save a byte in the SPI transfer protocol. */
    if (src > 0xff) {
        cmd |= 0x08;
    }
    /* LSB of address */
    addr = (src & 0xff);
    K60_ENTER_CRITICAL_REGION();
    spi_acquire_bus(spi_dev->spi);
    /* Write command and address */
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_CONT, &cmd, NULL, 1, 0);
    if (status < 0)
    {
        return status;
    }
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_CONT, &addr, NULL, 1, 0);
    if (status < 0)
    {
        return status;
    }
    /* Keep holding CS and read data */
    status = spi_transfer_blocking(spi_dev->spi, spi_dev->ctas, spi_dev->cs,
        SPI_TRANSFER_DONE, NULL, dst, 0, len);
    if (status < 0)
    {
        return status;
    }
    spi_release_bus(spi_dev->spi);
    K60_LEAVE_CRITICAL_REGION();
    /* status is 0 in Contiki if all went well. */
    return len;
}
