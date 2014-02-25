#include <string.h>
#include "stdio.h"
#include "rest-engine.h"
#include "er-coap.h"
#include "er-coap-observe.h"

static void res_get_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_periodic_handler(void);

static uint8_t message[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<senml bn=\"urn:dev:mac:0024befffe804ff1/\" bt=\"%lu\" ver=\"1\">\n"
    "\t<e n=\"pktid\" u=\"count\" v=\"%u\"/>\n"
    "\t<e n=\"rpm\" u=\"r/m\" v=\"336\"/>\n"
    "\t<e n=\"totrounds\" u=\"count\" v=\"1.7\"/>\n"
    "\t<e n=\"bearingtemp\" u=\"Cel\" v=\"40.7\"/>\n"
    "</senml>";

static uint8_t buf[500];

RESOURCE(res_wheel_loader,
    "title=\"Wheel loader bearing\"",
    res_get_handler,
    NULL,
    NULL,
    NULL);

static int32_t event_counter = 0;

static void
res_get_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  static uint32_t counter = 0;
  uint16_t size;
  uint16_t send_size;
  uint8_t* start = buf + *offset;

  if (*offset == 0)
  {
    // New request build message.
    sprintf((char*)buf, (char*)message, clock_seconds(),  counter++);
    printf("New message\n%sn", buf);
  }
  size = strlen(buf);

  printf("pref: %u offset: %lu\n", preferred_size, *offset);
  if (size > preferred_size)
  {
    send_size = ((size - *offset < preferred_size) ? size - *offset : preferred_size);
    *offset += send_size;
  }
  printf("Send size: %u\n", send_size);
  if(*offset >= size)
  {
    *offset = -1;
  }
  REST.set_header_content_type(response, REST.type.APPLICATION_XML);
  REST.set_header_max_age(response, 5);
  REST.set_response_payload(response, start, send_size);
}

static void
res_periodic_handler()
{
  ++event_counter;
  printf("Event\n");

  if(1) {
    /* Notify the registered observers which will trigger the res_get_handler to create the response. */
    REST.notify_subscribers(&res_wheel_loader);
  }
}
