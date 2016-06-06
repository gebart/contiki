/*
 * CoAP service that can be used to re-start to a bootloader.
 *
 * Copyright (C) 2013 Andrey Kruglyak
 *
 */

#ifndef __COAP_BOOTLOADER_H__
#define __COAP_BOOTLOADER_H__

#include "contiki.h"
#include "bootloader_api.h"

PROCESS_NAME(coap_bootloader_process);

void coap_bootloader_init(void);

#endif /* __COAP_BOOTLOADER_H__ */

