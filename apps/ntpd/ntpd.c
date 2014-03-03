/* 
 * SNTP implementation for Contiki
 *
 * Copyright (C) 2011 Anuj Sehgal <s.anuj@jacobs-university.de>
 *
 * This program is part of free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "ntpd.h"

#include <string.h>

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#if CONTIKI_TARGET_AVR_RAVEN
#include <avr/pgmspace.h>
#else
#define PROGMEM
#endif


static unsigned short SEND_INTERVAL=2;
static unsigned long StartTime=0;
static unsigned long CurrTime=0;
static unsigned long Offset=0;

static const char ntpmsg[] PROGMEM = {
  0x00000008U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
  0x00000000U, 0x00000000U, 0x00000000U, 0x00000000U,
};

static struct uip_udp_conn *ntp_conn;
PROCESS(ntpd_process, "ntpd");
/*---------------------------------------------------------------------------*/
unsigned long
getCurrTime(void)
{
  if(StartTime==0 && CurrTime==0)
    return 0;
  return (clock_seconds() - StartTime + CurrTime + Offset);
}
/*---------------------------------------------------------------------------*/
void
setOffset(unsigned long offset)
{
  Offset = offset;
}
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  if(uip_newdata() && (uip_datalen()==48)) {
    CurrTime = uip_ntohl(((struct ntpformat *)uip_appdata)->seconds) - NTP_EPOCH;
    StartTime = clock_seconds();
  }
}
/*---------------------------------------------------------------------------*/
static void
timeout_handler(void)
{
#if CONTIKI_TARGET_AVR_RAVEN
  char buf[48];
  strlcpy_P(buf, ntpmsg, 48);
  uip_udp_packet_send(ntp_conn, buf, 48);
#else
  uip_udp_packet_send(ntp_conn, ntpmsg, 48);
#endif
}
/*---------------------------------------------------------------------------*/
static void
set_connection_address(uip_ipaddr_t *ipaddr)
{
  uip_ip6addr(ipaddr,0xfdfd,0,0,0,0,0,0,0xff);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ntpd_process, ev, data)
{
  static struct etimer et;
  static uip_ipaddr_t ipaddr;

  PROCESS_BEGIN();
  PRINTF("ntpd process started\n");

  //set_connection_address(&ipaddr);

  /* find the IP of router */
  etimer_set(&et, CLOCK_SECOND);
  /*while(1){
    if(uip_ds6_defrt_choose()){
      uip_ipaddr_copy(&ipaddr, uip_ds6_defrt_choose());
      break;
    }
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_YIELD_UNTIL(etimer_expired(&et));
  }*/
  // Working ipv6 server
  //uip_ip6addr(&ipaddr, 0x2001, 0x14b8, 0x0, 0x3401,0x00,0x00,0x00,0x6);
  uip_ip6addr(&ipaddr, 0xfdfd, 0x0, 0x0, 0x0,0x00,0x00,0x00,0xff);

  /* new connection with remote host */
  ntp_conn = udp_new(&ipaddr, UIP_HTONS(NTPD_PORT), NULL);

  //etimer_set(&et, SEND_INTERVAL * CLOCK_SECOND);
  etimer_set(&et, 5 * CLOCK_SECOND);
  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&et)) {
      timeout_handler();
      
//      if((clock_seconds() > 4294967290U) || (clock_seconds() < 20)){
//	SEND_INTERVAL = 2 * CLOCK_SECOND;
//	etimer_set(&et, SEND_INTERVAL);
//      } else {
//	if(SEND_INTERVAL <= 512 && (getCurrTime() != 0)) {
//	  SEND_INTERVAL = 2 * SEND_INTERVAL;
//	}
//	etimer_set(&et, SEND_INTERVAL * CLOCK_SECOND);
//      }
      etimer_set(&et, CLOCK_SECOND*10);
    } else if(ev == tcpip_event) {
      tcpip_handler();
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
