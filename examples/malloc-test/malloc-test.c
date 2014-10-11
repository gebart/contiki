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
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 */

#include "contiki.h"

#include <stdio.h>              /* For printf() */
#include <stdint.h>             /* int16_t et al. */
#include <stdlib.h> /* malloc() */

extern int _heap_end;
extern int _heap_start;
extern int _end;

uint8_t* allocate_bytes(uint32_t nbytes)
{
  uint8_t* ret;
  printf("Allocating %u bytes...\n", nbytes);
  ret = (uint8_t*) malloc((size_t)nbytes);
  return ret;
}

/*---------------------------------------------------------------------------*/
PROCESS(lis3dh_poll_process, "LIS3DH poller process");
AUTOSTART_PROCESSES(&lis3dh_poll_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(lis3dh_poll_process, ev, data)
{
  static struct etimer et;
  static uint8_t* testptr;
  static uint8_t* oldptr;
  static uint32_t i = 0;


  PROCESS_BEGIN();

  printf("_heap_end    = 0x%08x\n", _heap_end);
  printf("&_heap_end   = 0x%08x\n", (uint32_t)(&_heap_end));
  printf("_heap_start  = 0x%08x\n", _heap_start);
  printf("&_heap_start = 0x%08x\n", (uint32_t)(&_heap_start));
  printf("_end         = 0x%08x\n", _end);
  printf("&_end        = 0x%08x\n", (uint32_t)(&_end));


  /* refresh every second */
  etimer_set(&et, CLOCK_SECOND);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Reset the etimer to trig again */
    etimer_reset(&et);

    testptr = allocate_bytes(i*10);

    printf("ptr = 0x%08x,\t incr = %d\n", (uint32_t)testptr, ((uint32_t)testptr - (uint32_t)oldptr));
    oldptr = testptr;
    ++i;
    if (i < 20)
    {
      printf("Freeing memory again\n");
      free(testptr);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
