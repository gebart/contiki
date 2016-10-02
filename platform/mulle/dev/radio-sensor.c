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

#include "net/netstack.h"
#include "lib/sensors.h"
#include "radio/rf212/at86rf212.h"
#include "dev/radio-sensor.h"
#include "dev/radio.h"

const struct sensors_sensor radio_sensor;
static int active;

/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  radio_value_t value;
  radio_result_t res;

  switch(type) {

  case RADIO_SENSOR_LAST_PACKET:
    // XXX: Why is there no documentation on the radio sensor in Contiki???
    // This will return the last received packet recorded LQI value
    res = NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_LINK_QUALITY, &value);
    break;

  case RADIO_SENSOR_LAST_VALUE:
  default:
    // TODO: Get _current_ RSSI??
    res = NETSTACK_RADIO.get_value(RADIO_PARAM_LAST_RSSI, &value);
    break;
  }
  if (res != RADIO_RESULT_OK) {
    return -1;
  }
  return value;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int c)
{
  if(type == SENSORS_ACTIVE) {
    active = c;
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return active;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(radio_sensor, RADIO_SENSOR,
               value, configure, status);
