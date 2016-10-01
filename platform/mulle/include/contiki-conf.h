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
 *         Contiki platform configuration file for the Mulle platform.
 * \author
 *         Joakim Gebart <joakim.gebart@eistec.se>
 *
 * \note These settings can be overridden by each project's project-conf.h
 *       (Set PROJECT_CONF_H in the project's Makefile)
 */
#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CCIF
#define CLIF

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int32_t s32_t;

typedef uint32_t clock_time_t;
typedef uint16_t uip_stats_t;

#define CLOCK_CONF_SECOND 64

typedef uint32_t rtimer_clock_t;
#define RTIMER_CLOCK_DIFF(a,b)     ((int32_t)((a) - (b)))

#define CFS_CONF_OFFSET_TYPE  long

#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */

/* Tell example applications that there are leds on board that can be controlled from software. */
#define PLATFORM_HAS_LEDS    1
/* Radio RSSI sensor using values from rf230 driver */
#define PLATFORM_HAS_RADIO   1
/* Battery voltage sensor using Mulle voltage API */
#define PLATFORM_HAS_BATTERY 1

/* Compatibility define for working with ugly msp430 specific parameter in UART API */
#define BAUD2UBR(b) b

#define LEDS_CONF_RED    (1 << 15)
#define LEDS_CONF_GREEN  (1 << 13)
#define LEDS_CONF_YELLOW (1 << 14)

/* Select UART modules to be claimed by the UART driver (dev/uart.c) below: */
#define UART0_CONF_ENABLE 0
#define UART1_CONF_ENABLE 1
#define UART2_CONF_ENABLE 0
#define UART3_CONF_ENABLE 0
#define UART4_CONF_ENABLE 0
#define UART5_CONF_ENABLE 0

/* TX buffer size for UART1 (must be a power of two <= 128) */
/* Try keeping this large (>= 64) to allow faster transfers for SLIP etc. */
#ifndef UART1_CONF_TXBUFSIZE
#define UART1_CONF_TXBUFSIZE 128
#endif

/* TX buffer size for not otherwise specified UART modules (must be a power of two <= 128) */
/* Keep this low to reduce memory consumption as one buffer per UART, even
 * unused ones, are statically allocated. */
#ifndef UART_CONF_DEFAULT_TXBUFSIZE
#define UART_CONF_DEFAULT_TXBUFSIZE 8
#endif

/* Disable slip-bridge implementation of putchar because it messes up newlib buffered stdio */
#define SLIP_BRIDGE_CONF_NO_PUTCHAR 1

#ifndef RF212_CONF_PHY_MODE
/* PHY mode is configured for 100 kbit/s data rate and following the 802.15.4 standard */
#define RF212_CONF_PHY_MODE         RF212_PHY_MODE_OQPSK_SIN_RC_100
#endif

/* IEEE802.15.4 frame version */
#undef FRAME802154_CONF_VERSION
#define FRAME802154_CONF_VERSION FRAME802154_IEEE802154E_2012

#if WITH_TSCH
/******************************* TSCH ***********************************/

/* TSCH packet calculations are done with respect to 250kbps data rates */
#undef RF212_CONF_PHY_MODE
#define RF212_CONF_PHY_MODE         RF212_PHY_MODE_OQPSK_SIN_250

/* Netstack layers */
#undef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC     tschmac_driver
#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC     nordc_driver
#undef NETSTACK_CONF_FRAMER
#define NETSTACK_CONF_FRAMER  framer_802154

/* TSCH and RPL callbacks */
#define RPL_CALLBACK_PARENT_SWITCH tsch_rpl_callback_parent_switch
#define RPL_CALLBACK_NEW_DIO_INTERVAL tsch_rpl_callback_new_dio_interval
#define TSCH_CALLBACK_JOINING_NETWORK tsch_rpl_callback_joining_network
#define TSCH_CALLBACK_LEAVING_NETWORK tsch_rpl_callback_leaving_network

#define TSCH_CONF_DEFAULT_TIMESLOT_LENGTH 15000

#define TSCH_LOG_CONF_LEVEL 0

/* Delay between GO signal and SFD */
/* Sleep to tx => 540 us (measured time) + 4 bytes preamble  + 1 byte SFD*/
#define RADIO_DELAY_BEFORE_TX ((unsigned)US_TO_RTIMERTICKS(540+2*16*(4+1)))

/* Delay between GO signal and start listening */
/* Sleep to rx => 460 (measured time) */
#define RADIO_DELAY_BEFORE_RX ((unsigned)US_TO_RTIMERTICKS(460))
//#define RADIO_DELAY_BEFORE_RX 0

/* Delay between the SFD finishes arriving and it is detected in software */
/* IRQ delay 8 us + PHR byte */
#define RADIO_DELAY_BEFORE_DETECT ((unsigned)US_TO_RTIMERTICKS((8+2*16*1)))

#define RF_CHANNEL                  1 /* not needed */
#define TSCH_CONF_DEFAULT_HOPPING_SEQUENCE (uint8_t[]){ 1 }

#define RF212_CONF_HARDWARE_ACK     0 /* TSCH sends acks by itself */
#define RF212_CONF_SEND_ON_CCA 0
#define RF212_CONF_CCA_RETRIES   0
#define RF212_CONF_AUTORETRIES      0
#define RF212_CONF_FRAME_FILTERING 0

/* TSCH uses polling, but we dont do real polling yet so set buffers to 1 */
#define RF212_CONF_RX_BUFFERS       1

/******************************* CONTIKIMAC ***********************************/
#elif CONTIKIMAC

/* CONTIKIMAC has only been tested with O-QPSK-100 */
#undef RF212_CONF_PHY_MODE
/* PHY mode is configured for 100 kbit/s data rate and following the 802.15.4 standard */
#define RF212_CONF_PHY_MODE         RF212_PHY_MODE_OQPSK_SIN_RC_100

/**
 * BPSQ: 1 Symbol = 1 Bit => 8 Symbols = 1 Byte
 * O-QPSK: 1 Symbol = 4 Bits => 2 Symbols = 1 Byte
 *
 * Tl   (ms): The transmission time of the longest possible packet (127 bytes of payload)
 * Ta   (ms): The time between receiving a packet and sending the acknowledgment packet, 12 symbols for 802.15.4
 * Td   (ms): The time required for successfully detecting an acknowledgment from the receiver, this can be done after SHR+PHR
 * Sp   (us): Symbol period (us) (table 7.2)
 * Tr   (ms): CCA measurement time, 8 symbols
 * Tc   (ms): Interval between CCA measurements
 * Ts   (ms): The transmission time of the shortest packet
 * Ti   (ms): The interval between each packet transmission.
 * Pbytes   : Minimum packet size allowed to be sent
 *
 * The parameters must fulfill the following condition:
 *
 * Ta + Td < Ti < Tc < Tc + 2Tr < Ts.
 *
 * NOTE: The RF212 datasheet specifies 300 us for SHR with O-QPSK-(100, 200, 400) but calculating it gives 400,
 *       10 symbols with a speed of 40 = 400 us. 400 is used here.
 *
 * MODE         Sp      Tl      Ta        Td      Tr      min(Ts)  Ti   Tc    Pbytes
 * BPSQ-20      50      53.2    0.6       2.4     0.4     3.8      3.5  4     6
 * BPSQ-40      25      26.6    0.3       1.2     0.2     1.9      3    3.5   13.5
 * O-QPSK-100   40      10.64   0.48      0.48    0.32    1.6      1    1.1   15.75
 * O-QPSK-250   16      4.254   0.192     0.192   0.128   0.64     0.5  0.6   21
 * O-QPSK-200   40      5.56    0.48      0.48    0.32    1.6      1    1.1   31.5
 * O-QPSK-400   40      3.02    0.48      0.48    0.32    1.6
 * O-QPSK-500   16      2.224   0.192     0.192   0.128   0.64
 * O-QPSK-1000  16      1.208   0.192     0.192   0.128   0.64
 *
 * NOTE: Ti and Tc are in some cases tuned manually for better performance.
 *       In general increasing CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED (=> larger Tl)
 *       gives less PL.
 */

#if RF212_CONF_PHY_MODE == RF212_PHY_MODE_BPSK_20
#define CONTIKIMAC_Ti 3.5
#define CONTIKIMAC_Tc 4
#define CONTIKIMAC_Tr 0.4
#define CONTIKIMAC_Tl 53.2
#define CONTIKIMAC_Td 2.4
#elif RF212_CONF_PHY_MODE == RF212_PHY_MODE_BPSK_40
#define CONTIKIMAC_Ti 3
#define CONTIKIMAC_Tc 3.5
#define CONTIKIMAC_Tr 0.2
#define CONTIKIMAC_Tl 26.6
#define CONTIKIMAC_Td 1.2
#elif RF212_CONF_PHY_MODE == RF212_PHY_MODE_OQPSK_SIN_RC_100
/* Values increased after measurements with oscilloscope */
#define CONTIKIMAC_Ti 1.2 // Manually increased from 1.0
#define CONTIKIMAC_Tc 4.1 // Manually increased from 1.1
#define CONTIKIMAC_Tr 0.32
#define CONTIKIMAC_Tl (10.64 * 1.5) // Manually increased with 50%
#define CONTIKIMAC_Td 0.48
#elif RF212_CONF_PHY_MODE == RF212_PHY_MODE_OQPSK_SIN_RC_200
#define CONTIKIMAC_Ti 1
#define CONTIKIMAC_Tc 1.1
#define CONTIKIMAC_Tr 0.32
#define CONTIKIMAC_Tl (5.56 + 0.5) // Manually increased Tl for better stability
#define CONTIKIMAC_Td 0.48
#elif RF212_CONF_PHY_MODE == RF212_PHY_MODE_OQPSK_SIN_250
#define CONTIKIMAC_Ti 0.5
#define CONTIKIMAC_Tc 0.6
#define CONTIKIMAC_Tr 0.128
#define CONTIKIMAC_Tl (4.254 + 0.5) // Manually increased Tl for better stability
#define CONTIKIMAC_Td 0.192
#else
#error "CONTIKIMAC does not support the specified radio speed"
#endif

#undef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC         contikimac_driver

/*
 * Contikimac strobes by itself.
 */
#define RF212_CONF_HARDWARE_ACK     1
#define RF212_CONF_SEND_ON_CCA 1
#define RF212_CONF_CCA_RETRIES   0
#define RF212_CONF_AUTORETRIES      0
#define RF212_CONF_FRAME_FILTERING 1

#define RDC_CONF_HARDWARE_CSMA    1
#define RDC_CONF_HARDWARE_ACK    1
#define NETSTACK_CONF_RDC_CHANNEL_CHECK_RATE   8
#define CSMA_CONF_MAX_MAC_TRANSMISSIONS 1
#define SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS 1
/* Going from 2 to 3 CCA checks gives almost zero packetloss. */
#define CONTIKIMAC_CONF_CCA_COUNT_MAX 3

// Ti
#define CONTIKIMAC_CONF_INTER_PACKET_INTERVAL \
    ((RTIMER_ARCH_SECOND) / (1000 / (CONTIKIMAC_Ti)))
// Tc
#define CONTIKIMAC_CONF_CCA_SLEEP_TIME \
    ((RTIMER_ARCH_SECOND) / (1000 / (CONTIKIMAC_Tc)))
// Tr
#define CONTIKIMAC_CONF_CCA_CHECK_TIME \
    ((RTIMER_ARCH_SECOND) / (1000 / (CONTIKIMAC_Tr)))
// Time to send 2 packets (tl) + one wait between packets (Ti)
#define CONTIKIMAC_CONF_LISTEN_TIME_AFTER_PACKET_DETECTED \
    (((CONTIKIMAC_CONF_INTER_PACKET_INTERVAL) + 2 * ((RTIMER_ARCH_SECOND) / (1000 / (CONTIKIMAC_Tl)))))
// Td
#define CONTIKIMAC_CONF_AFTER_ACK_DETECTECT_WAIT_TIME \
    ((RTIMER_ARCH_SECOND) / (1000 / (CONTIKIMAC_Td)))

/* Phase optimization seem to cause problems (drifting clocks?) */
#define CONTIKIMAC_CONF_WITH_PHASE_OPTIMIZATION 0

/* Below are TSCH parameters, not used by ContikiMAC, but defined to avoid build
 * errors when including the TSCH module, which simplifies the Makefile */
#define RADIO_DELAY_BEFORE_TX 0
#define RADIO_DELAY_BEFORE_RX 0
#define RADIO_DELAY_BEFORE_DETECT 0

#else
/******************************** Nullrdc *************************************/
#ifndef NETSTACK_CONF_RDC
#define NETSTACK_CONF_RDC           nullrdc_driver
#endif /* NETSTACK_CONF_RDC */

#define RDC_CONF_HARDWARE_ACK     1
/* TX routine does automatic cca and optional backoffs */
#define RDC_CONF_HARDWARE_CSMA    1
#define RF212_CONF_HARDWARE_ACK     1
#define RF212_CONF_SEND_ON_CCA 1
#define RF212_CONF_CCA_RETRIES   0
#define RF212_CONF_AUTORETRIES      3

/* Below are TSCH parameters, not used by nullrdc, but defined to avoid build
 * errors when including the TSCH module, which simplifies the Makefile */
#define RADIO_DELAY_BEFORE_TX 0
#define RADIO_DELAY_BEFORE_RX 0
#define RADIO_DELAY_BEFORE_DETECT 0

#endif /* TSCH */

#ifndef NETSTACK_CONF_NETWORK
#define NETSTACK_CONF_NETWORK       sicslowpan_driver
#endif /* NETSTACK_CONF_NETWORK */

#ifndef NETSTACK_CONF_MAC
#define NETSTACK_CONF_MAC           csma_driver
#endif /* NETSTACK_CONF_MAC */


#ifndef NETSTACK_CONF_FRAMER
#if NETSTACK_CONF_WITH_IPV6
#define NETSTACK_CONF_FRAMER  framer_802154
#else /* NETSTACK_CONF_WITH_IPV6 */
#define NETSTACK_CONF_FRAMER  contikimac_framer
#endif /* NETSTACK_CONF_WITH_IPV6 */
#endif /* NETSTACK_CONF_FRAMER */

#ifndef NETSTACK_CONF_RADIO
#define NETSTACK_CONF_RADIO         rf212_driver
#endif /* NETSTACK_CONF_RADIO */

#define SICSLOWPAN_CONF_MAXAGE      1

#ifndef RF212_CONF_RX_BUFFERS
#define RF212_CONF_RX_BUFFERS       10
#endif

#define LINKADDR_CONF_SIZE          8

#ifndef UIP_CONF_BUFFER_SIZE
#define UIP_CONF_BUFFER_SIZE        2048
#endif /* UIP_CONF_BUFFER_SIZE */

#ifndef QUEUEBUF_CONF_NUM
#define QUEUEBUF_CONF_NUM           16
#endif /* QUEUEBUF_CONF_NUM */

#ifndef UIP_CONF_ROUTER
#define UIP_CONF_ROUTER                 1
#endif /* UIP_CONF_ROUTER */

#define SICSLOWPAN_CONF_COMPRESSION         SICSLOWPAN_COMPRESSION_HC06
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS   0
#define SICSLOWPAN_CONF_FRAG                1

#define UIP_CONF_UDP                1
#define UIP_CONF_UDP_CHECKSUMS      1

/* configure number of neighbors and routes */
#define UIP_CONF_DS6_NBR_NBU        15
#define UIP_CONF_DS6_ROUTE_NBU      15
#define NEIGHBOR_CONF_MAX_NEIGHBORS 15

#ifndef UIP_CONF_ND6_SEND_RA
#define UIP_CONF_ND6_SEND_RA            0
#endif
#ifndef UIP_CONF_ND6_REACHABLE_TIME
/* RFC 4861 default value (REACHABLE_TIME): 30000 ms */
#define UIP_CONF_ND6_REACHABLE_TIME 600000
#endif
#ifndef UIP_CONF_ND6_RETRANS_TIMER
/* RFC 4861 default value (RETRANS_TIMER): 1000 ms */
#define UIP_CONF_ND6_RETRANS_TIMER  10000
#endif

//~ #define NETSTACK_CONF_WITH_IPV6         1
#define UIP_CONF_IPV6_QUEUE_PKT         1
#define UIP_CONF_IPV6_CHECKS            1
#define UIP_CONF_IPV6_REASSEMBLY        0
#define UIP_CONF_NETIF_MAX_ADDRESSES    3
#define UIP_CONF_ND6_MAX_PREFIXES       3
#define UIP_CONF_ND6_MAX_NEIGHBORS      5
#define UIP_CONF_ND6_MAX_DEFROUTERS     2
#define UIP_CONF_IP_FORWARD             0

#define UIP_CONF_ICMP_DEST_UNREACH  1
#define UIP_CONF_DHCP_LIGHT
#define UIP_CONF_LL_802154          1

#ifndef UIP_CONF_LLH_LEN
#define UIP_CONF_LLH_LEN            0
#endif /* UIP_CONF_LLH_LEN */

#ifndef UIP_CONF_RECEIVE_WINDOW
#define UIP_CONF_RECEIVE_WINDOW     60
#endif /* UIP_CONF_RECEIVE_WINDOW */

#define UIP_CONF_TCP_MSS            48
#define UIP_CONF_MAX_CONNECTIONS    4
#define UIP_CONF_MAX_LISTENPORTS    8
#define UIP_CONF_UDP_CONNS          15
#define UIP_CONF_FWCACHE_SIZE       15
#define UIP_CONF_BROADCAST          1
#define UIP_CONF_UDP                1
#define UIP_CONF_UDP_CHECKSUMS      1
#define UIP_CONF_PINGADDRCONF       0
#define UIP_CONF_LOGGING            0

#define UIP_CONF_TCP_SPLIT          1
#ifndef UIP_CONF_TCP
#define UIP_CONF_TCP                0
#endif

#define PACKETBUF_CONF_ATTRS_INLINE 1

#ifndef RF_CHANNEL
#define RF_CHANNEL                  0
#endif /* RF_CHANNEL */

#ifndef IEEE802154_CONF_PANID
#define IEEE802154_CONF_PANID       0x777
#endif /* IEEE802154_CONF_PANID */

#ifndef COAP_MAX_OPEN_TRANSACTIONS
/* Multiplies with chunk size, be aware of memory constraints. */
#define COAP_MAX_OPEN_TRANSACTIONS   11
#endif /* COAP_MAX_OPEN_TRANSACTIONS */

/* Must be <= open transaction number. */
#define COAP_MAX_OBSERVERS      (COAP_MAX_OPEN_TRANSACTIONS - 1)

/* Tell Contiki that we are not using PETSCII (retarded default) */
#define WITH_ASCII 1

/* RIOT compatibility, currently missing MRHOF and HBH option support */
#define RPL_CONF_OF rpl_of0
#define RPL_CONF_INSERT_HBH_OPTION 0

#ifndef BOARD_STRING
#define BOARD_STRING "Mulle"
#endif

/* LWM2M default settings */
#ifndef LWM2M_DEVICE_MODEL_NUMBER
#define LWM2M_DEVICE_MODEL_NUMBER BOARD_STRING
#endif
#ifndef LWM2M_DEVICE_MANUFACTURER
#define LWM2M_DEVICE_MANUFACTURER "Eistec AB"
#endif
#ifndef LWM2M_DEVICE_FIRMWARE_VERSION
#define LWM2M_DEVICE_FIRMWARE_VERSION "0.0.0"
#endif
#ifndef LWM2M_DEVICE_SERIAL_NO
#define _stringify2(x) #x
#define _stringify_macro(x) _stringify2(x)
#define LWM2M_DEVICE_SERIAL_NO    _stringify_macro(MULLE_SERIAL)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CONTIKI_CONF_H__ */
