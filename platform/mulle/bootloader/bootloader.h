/**
 * \file
 *         Bootloader internal routines header file.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_

#include <stdint.h>
#include "bootloader-arch.h"
#include "bootloader_app.h"

#define CODE_NEW_APP {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8}
#define CODE_USER_VERIFICATION {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8}
#define CODE_BOOTLOADER_VERIFICATION {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8}

/**
 * Bootloader startup sequence. This function should be called at the beginning
 * of main. If the user application should be booted one doesn't want to init
 * unnecessary functionality.
 */
void bootloader_startup(void);

void bootloader_run_app(void);

// Function provided by target platform
/**
 * Erases program flash memory.
 */
int bl_arch_flash_erase(void);

/**
 * Writes to program flash memory.
 */
int bl_arch_flash_write(unsigned long addr, uint8_t* buf, unsigned long len);

/**
 * Boots user application.
 */
void bl_arch_run_app(void);
/**
 * Checks whether the user application should be executed.
 * @return 1 if the user application should execute else 0.
 */
int bl_arch_is_run_app(void);

/**
 * Sets necessary flags to go into user application.
 */
int bl_arch_set_run_app(void);

/**
 * Sets necessary flags to go into bootloader mode.
 */
int bl_arch_set_run_bootloader(void);

/**
 * Reads/writes non volatile memory used by bootloader.
 */
int bl_arch_read_bl_mem(uint32_t addr, uint8_t* buf, uint32_t len);
int bl_arch_write_bl_mem(uint32_t addr, uint8_t* buf, uint32_t len);

/**
 * Setups necessary functions on target platform to be able to verify user app.
 */
void arch_bl_start_verify(void);

/**
 * Indicates that before the last reboot verification was attempted.
 */
int bl_arch_tried_verify(void);

/**
 * Checks whether verification is running.
 */
int bl_arch_is_verifying(void);
#endif /* BOOTLOADER_H_ */
