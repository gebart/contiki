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
 *         Adam Dunkels <adam@sics.se>
 */

#define NUM_ACC_AXES 3
#define EXAMPLE_POLL_FREQUENCY 8

#include "contiki.h"

#include "hih6130.h"
#include "i2c.h"

#include <stdio.h>              /* For printf() */
#include <stdint.h>             /* int16_t et al. */
#include <stdlib.h>             /* abs */
#include <math.h>               /* modff */

#define HIH6130_ADDR 0x27
#define HIH6130_BUS I2C_0

/*---------------------------------------------------------------------------*/
PROCESS(hih6130_process, "HIH6130 poller process");
AUTOSTART_PROCESSES(&hih6130_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hih6130_process, ev, data)
{
  static struct etimer et;
  int16_t acc_buffer[NUM_ACC_AXES];
  int16_t temperature;
  static hih6130_t dev;
  static int status;

  PROCESS_BEGIN();

  puts("HIH6130 test");

  /* initialize the LIS3DH hardware */
  printf("Initializing I2C_%i... ", HIH6130_BUS);
  i2c_init_master(HIH6130_BUS, I2C_SPEED_FAST);
  
  printf("Initializing HIH6130 sensor at I2C_%i, address 0x%02x... ",
      HIH6130_BUS, HIH6130_ADDR);
  hih6130_init(&dev, HIH6130_BUS, HIH6130_ADDR);

  /* Request a new measurement */
  status = hih6130_measurement_request(&dev);
  if (status < 0) {
      printf("Communication error: %d\n", status);
  }

  /* Poll at 8Hz */
  etimer_set(&et, CLOCK_SECOND / EXAMPLE_POLL_FREQUENCY);

  while(1) {
    static float hum = 0.f;
    static float temp = 0.f;
    static float integral = 0.f;
    static float fractional;
    
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    /* Reset the etimer to trig again */
    etimer_reset(&et);

    /* Read values */
    status = hih6130_get_humidity_temperature_float(&dev, &hum, &temp);
    if (status < 0) {
        printf("Communication error: %d\n", status);
        continue;
    } else if (status == 1) {
        puts("Stale values");
    }

    /* Request a new measurement, the device takes > 37 ms to sample the sensor */
    status = hih6130_measurement_request(&dev);
    if (status < 0) {
        printf("Communication error: %d\n", status);
        continue;
    }

    /* Several platforms usually build with nano.specs, (without float printf) */
    /* Split value into two integer parts for printing. */
    fractional = modff(hum, &integral);
    printf("humidity: %4d.%04u %%",
        (int)integral, (unsigned int)abs(fractional * 10000.f));
    fractional = modff(temp, &integral);
    printf("  temperature: %4d.%04u C\n",
        (int)integral, (unsigned int)abs(fractional * 10000.f));
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
