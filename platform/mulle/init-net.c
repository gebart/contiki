/*
 * Copyright (c) 2014-2015 Eistec AB.
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
 *         Network initialization for the Mulle platform
 * \author
 *         Henrik Mäkitaavola <henrik.makitaavola@gmail.com>
 *         Joakim Gebart <joakim.gebart@eistec.se
 */

#include <stdio.h>
#include <string.h>
#include "queuebuf.h"
#include "K60.h"

#include "contiki.h"
#include "at86rf212.h"
#include "net/netstack.h"
#include "net/mac/frame802154.h"
#if WITH_IPSO
#include "lwm2m-engine.h"
#include "ipso-objects.h"
#endif

#define DEBUG 1
#include "net/ip/uip-debug.h"

#if NETSTACK_CONF_WITH_IPV6
#include "net/ipv6/uip-ds6.h"
#endif /* NETSTACK_CONF_WITH_IPV6 */

union {
    uint64_t u64;
    uint32_t u32[sizeof(uint64_t) / sizeof(uint32_t)];
    uint16_t u16[sizeof(uint64_t) / sizeof(uint16_t)];
    uint8_t  u8[sizeof(uint64_t) / sizeof(uint8_t)];
} mulle_eui64;

char mulle_eui64_str[sizeof(mulle_eui64) * 2 + 1];

/** @brief Simple hash function used for generating link-local IPv6 and EUI64
 *         (64 bit) from CPUID (128 bit)
 *
 * @see http://www.cse.yorku.ca/~oz/hash.html */
static uint32_t
djb2_hash(const uint8_t *buf, size_t len)
{
    uint32_t hash = 5381;
    size_t i;

    for (i = 0; i < len; ++i)
        hash = ((hash << 5) + hash) + buf[i]; /* hash * 33 + c */

    return hash;
}

/*---------------------------------------------------------------------------*/
static void
set_rime_addr(void)
{
  linkaddr_t addr;
  unsigned int i;

  /* memset(&addr, 0x65, sizeof(linkaddr_t)); */
  memcpy(addr.u8, mulle_eui64.u8, sizeof(addr.u8));

  linkaddr_set_node_addr(&addr);
  PRINTF("Rime started with address ");
  PRINTF("%d", addr.u8[0]);
  for(i = 1; i < sizeof(addr.u8); i++) {
    PRINTF(".%d", addr.u8[i]);
  }
  PRINTF("\n");
}
/*---------------------------------------------------------------------------*/
void
init_net(void)
{
#if WITH_SLIP
  /* Use fixed address for the border router. */
  mulle_eui64.u32[0] = 0x00000000;
  mulle_eui64.u32[1] = 0x00000000;
  mulle_eui64.u8[7] = 0x01;
#else
  mulle_eui64.u32[0] = djb2_hash((const uint8_t *)&(SIM->UIDH), 8); /* Use SIM_UIDH, SIM_UIDMH for first half */
  mulle_eui64.u32[1] = djb2_hash((const uint8_t *)&(SIM->UIDML), 8); /* Use SIM_UIDML, SIM_UIDL for second half */
#endif
  mulle_eui64.u8[0] |= 0x02; /* Set the Local/Universal bit to Local */
#if NETSTACK_CONF_WITH_IPV6
  set_rime_addr();
  NETSTACK_RADIO.init();
  /* Radio needs to be on when setting settings. */
  NETSTACK_RADIO.on();
  do {
    uint8_t longaddr[8];
    uint16_t shortaddr;

    shortaddr = (linkaddr_node_addr.u8[0] << 8) +
      linkaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);
    at86rf212_set_pan_addr(IEEE802154_CONF_PANID, shortaddr, longaddr);
    PRINTF("PAN ID: 0x%04X\n", IEEE802154_CONF_PANID);
    PRINTF("RF channel: %u\n", RF_CHANNEL);
    PRINTF("longaddr: ");
    for(int i = 0; i < 7; ++i) {
      PRINTF("%02x-", longaddr[i]);
    }
    PRINTF("%02x\n", longaddr[7]);
  } while(0);
  NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, RF_CHANNEL);

  snprintf(mulle_eui64_str, sizeof(mulle_eui64_str), "%02x%02x%02x%02x%02x%02x%02x%02x",
    mulle_eui64.u8[0],
    mulle_eui64.u8[1],
    mulle_eui64.u8[2],
    mulle_eui64.u8[3],
    mulle_eui64.u8[4],
    mulle_eui64.u8[5],
    mulle_eui64.u8[6],
    mulle_eui64.u8[7]);

  PRINTF("Using EUI64 string: %s\n", mulle_eui64_str);

  memcpy(&uip_lladdr.addr, mulle_eui64.u8, sizeof(uip_lladdr.addr));

  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_LLSEC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  PRINTF("%s %s, channel check rate %d Hz, radio channel %d\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (NETSTACK_RDC.channel_check_interval() == 0 ? 1 :
                         NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);

  process_start(&tcpip_process, NULL);

  PRINTF("Radio PHY mode: 0x%02x\n", RF212_CONF_PHY_MODE);

  PRINTF("Tentative link-local IPv6 address ");
  do {
    uip_ds6_addr_t *lladdr;
    lladdr = uip_ds6_get_link_local(-1);
    PRINT6ADDR(&lladdr->ipaddr);
  } while(0);
  PRINTF("\n");

  if(!UIP_CONF_IPV6_RPL) {
    uip_ipaddr_t ipaddr;
    uip_ip6addr(&ipaddr, 0xfdfd, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
    uip_ds6_addr_add(&ipaddr, 0, ADDR_TENTATIVE);
    PRINTF("Tentative global IPv6 address ");
    PRINT6ADDR(&ipaddr);
    PRINTF("\n");
  }

#if WITH_IPSO
  PRINTF("Running with IPSO object support\n");
  /* Initialize the OMA LWM2M engine */
  lwm2m_engine_init();

  /* Register default LWM2M objects */
  lwm2m_engine_register_default_objects();

  /* Register default IPSO objects */
  ipso_objects_init();

  /* Defer registration until user application approves */
  lwm2m_engine_use_bootstrap_server(0);
  lwm2m_engine_use_registration_server(0);

#ifdef LWM2M_SERVER_ADDRESS
  do {
    uip_ipaddr_t addr;
    if(uiplib_ipaddrconv(LWM2M_SERVER_ADDRESS, &addr)) {
      lwm2m_engine_register_with_bootstrap_server(&addr, 0);
      lwm2m_engine_register_with_server(&addr, 0);
    }
  } while(0);
#endif /* LWM2M_SERVER_ADDRESS */
#endif /* WITH_IPSO */

#else /* If no radio stack should be used only turn on radio and set it to sleep for minimal power consumption */
  rf212.init();
  rf212.off();
#endif /* NETSTACK_CONF_WITH_IPV6 */
}
