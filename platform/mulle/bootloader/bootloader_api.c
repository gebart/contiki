/**
 * \file
 *         Implementation of bootloader routines intended to be used by user
 *         application.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#include "bootloader.h"
#include "bootloader-arch.h"
// DEBUGGING
#define DEBUG 0
#if DEBUG
#include "stdio.h"
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

void bootloader_run(void)
{
  PRINTF("Run bootloader\n");
  bl_arch_set_run_bootloader();
  REBOOT();
}

int bootloader_is_verifying(void)
{
  return bl_arch_is_verifying();
}

int bootloader_write_user_verification(uint8_t code[8])
{
  return bl_app_set_user_verification(code);
}
