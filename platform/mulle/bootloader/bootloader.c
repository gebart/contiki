/**
 * \file
 *         Bootloader internal routines.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#include "bootloader-arch.h"
#include "bootloader.h"
#include "contiki.h"
#include "dev/leds.h"
#include "autostart.h"
#include "udelay.h"
PROCESS_NAME(bootloader_process);
AUTOSTART_PROCESSES(&bootloader_process);

// DEBUGGING
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static void leds_glow(uint8_t a, uint8_t b)
{
  int i;
  for (i = 1000; i > 0; i -= 4) {
    leds_off(LEDS_ALL);
    leds_on(a);
    udelay(i);
    leds_off(LEDS_ALL);
    leds_on(b);
    udelay(1000-i);
  }
}

static void startup_leds(void)
{
  int i;
  uint8_t output = 0x7;
  for (i = 3; i; i--, output >>= 1 )
  {
    leds_glow(output, output>>1);
  }
}

static void boot_user_app(int verify)
{
  PRINTF("Launching application\n");
  startup_leds();
  leds_off(LEDS_ALL);
  if (verify)
  {
    //PRINTF("Start with verification\n");
    //arch_bl_start_verify();
  }
  bl_arch_run_app();
  while(1); // Should not get here
}

void bootloader_startup(void)
{
  PRINTF("******Bootloader startup\n");

  if(bl_arch_is_run_app() && !bl_app_is_empty())
  {
    // Run user application
    boot_user_app(0);
  }

  PRINTF("Start bootloader\n");
}

void bootloader_run_app(void)
{
  bl_arch_set_run_app();
  REBOOT();
}
