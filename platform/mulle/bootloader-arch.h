/**
 * \file
 *         Arch specific settings for the bootloader.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 */

#ifndef BOOTLOADER_ARCH_H_
#define BOOTLOADER_ARCH_H_

#include "K60.h"
#include "uart.h"
#include "udelay.h"

#define BL_MEM_FLASH_START 0x40000ul
#define USER_APP_FLASH_START (BL_MEM_FLASH_START + 0x100) // Start address of user application in flash

#define REBOOT() SCB->AIRCR =  (0x5FA0000 | SCB_AIRCR_SYSRESETREQ_Msk)
#endif /* BOOTLOADER_ARCH_H_ */
