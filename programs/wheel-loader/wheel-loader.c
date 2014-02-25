#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "rest-engine.h"

#define DEBUG   DEBUG_PRINT
#include "net/uip-debug.h"

/*
 * Resources to be activated need to be imported through the extern keyword.
 * The build system automatically compiles the resources in the corresponding sub-directory.
 */
extern resource_t res_wheel_loader;

PROCESS(wheel_loader, "Wheel Loader");
AUTOSTART_PROCESSES(&wheel_loader);

PROCESS_THREAD(wheel_loader, ev, data)
{
  PROCESS_BEGIN();

  PROCESS_PAUSE();

  PRINTF("Starting Wheel Loader Process\n");

  /* Initialize the REST engine. */
  rest_init_engine();


  rest_activate_resource(&res_wheel_loader, "wheel_loader");


  /* Define application-specific events here. */
  while(1)
  {
    PROCESS_WAIT_EVENT();
  }

  PROCESS_END();
}
