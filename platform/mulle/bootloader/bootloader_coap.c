/**
 * \file
 *         Bootloader with reprogramming capabilities through CoAP resources.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#define DEBUG 0
#if DEBUG
#include "stdio.h"
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#include <string.h>

#include "contiki.h"
#include "coap_reprogram.h"
#include "rest-engine.h"

/*---------------------------------------------------------------------------*/
PROCESS(bootloader_process, "Bootloader process");
AUTOSTART_PROCESSES(&bootloader_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(bootloader_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("Start Bootloader CoAP Resource\n");
  rest_init_engine();
  coap_reprogram_init();

  PROCESS_END();
}
