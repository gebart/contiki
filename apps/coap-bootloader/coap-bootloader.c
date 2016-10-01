/*
 * CoAP service that can be used to re-start to the bootloader.
 *
 * Copyright (C) 2013 Andrey Kruglyak
 *
 */

#include "coap-bootloader.h"
#include <string.h>
#include "contiki.h"
#include "er-coap.h"

#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

static struct ctimer timer[1];

static void run_bootloader(void* data)
{
  bootloader_run();
}

/* ----- CoAP service ----- */

void run_bootloader_handler(void* request, void* response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset);

RESOURCE(run_bootloader_service,
    "title=\"Run bootloader\"",
    NULL,
    NULL,
		run_bootloader_handler,
    NULL);

void
run_bootloader_handler(void* request, void* response, uint8_t *buffer,
    uint16_t preferred_size, int32_t *offset)
{
  if (REST.get_method_type(request) == METHOD_PUT) {
    const uint8_t* payload;
    char text[100];
    int len;
    len = REST.get_request_payload(request, &payload);
    if (len < 0) {
      /* Should never be possible */
      /* er-coap returns 0 on no payload */
      return;
    }
    else if (len > sizeof(text)) {
      len = sizeof(text);
    }
    memcpy(text, payload, len);
    text[len] = '\0';
    if (text[0] != '1') {
      REST.set_response_status(response, REST.status.BAD_REQUEST);
    } else {
      REST.set_response_status(response, REST.status.CHANGED);
      ctimer_set(timer, CLOCK_SECOND*2, run_bootloader, 0); // Reboot in 2 seconds, gives time for response
    }
  }
}

/* ----- Contiki process ----- */

PROCESS(coap_bootloader_process, "CoAP run bootloader process");

void coap_bootloader_init(void)
{
  rest_activate_resource(&run_bootloader_service, "run_bootloader");
}

PROCESS_THREAD(coap_bootloader_process, ev, data)
{
  PROCESS_BEGIN();
  // This requires that rest_init_engine() has already been invoked.
  coap_bootloader_init();
  PROCESS_END();
}
