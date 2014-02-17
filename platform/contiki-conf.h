#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

#define CCIF
#define CLIF

typedef unsigned short uip_stats_t;
typedef unsigned long  clock_time_t;
typedef unsigned long  bool_t;
typedef uint8_t   u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int32_t  s32_t;

#define CLOCK_CONF_SECOND 64

#define CFS_CONF_OFFSET_TYPE	long

#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif

#define printf(...)

#endif /* __CONTIKI_CONF_H__ */
