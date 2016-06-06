/**
 * \file
 *         Implementation of CoAP Routines for reprogramming of flash memory
 *         and startup of new application.
 * \author
 *         Henrik Mäkitaavola <henrik@makitaavola.se>
 *
 *         Copyright (c) 2013, Eislab, Luleå University of Technology
 */

#define DEBUG 0
#if DEBUG
#include "stdio.h"
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#include "er-coap.h"

#include "bootloader.h"
#include "bootloader_app.h"
#include "bootloader_api.h"
#include "bootloader-arch.h"
#include <string.h>

#define EMPTY_TEXT "Application space is empty"
#define NON_EMPTY_TEXT "Application space is not empty"
#define BAD_INPUT_TEXT "Bad input"
#define NODE_BUSY_TEXT "Node is busy"
#define APP_VALID_TEXT "Application is valid"
#define APP_NOT_VALID_TEXT "Application is NOT valid"
#define RECEIVING_APP_TEXT "Already receiving application"
#define NOT_EXPECTING_DATA_TEXT "We are not expecting application data"
#define DATA_LARGER_APP_TEXT "Received data is larger than expected"
#define UNABLE_TO_ERASE_TEXT "Unable to erase flash"
#define UNABLE_TO_WRITE_FLASH_TEXT "Unable to write data to flash"
#define APP_VERIFIED_TEXT "Application is ready"
#define APP_NOT_VERIFIED_TEXT "Application is NOT ready"

enum
{
  S_IDLE,
  S_RUN,
  S_RECEIVING
};
static uint8_t state = S_IDLE;
static struct ctimer rec_timer[1];
//uint8_t app_buffer[514];
//uint16_t app_buffer_pos;
//uint32_t addr;
uint32_t next_offset;

PROCESS(coap_rep_process, "CoAP reprogram process");

/*---------------------------------------------------------------------------*/
void rec_timeout(void* data)
{
  PRINTF("Reception timedout\n");
  state = S_IDLE;
}
/*---------------------------------------------------------------------------*/
void rep_upload_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
RESOURCE(rep_upload, "title=\"Upload new application\"", NULL, NULL, rep_upload_handler, NULL);
void
rep_upload_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  if (state != S_IDLE && state != S_RECEIVING)
  {
    REST.set_response_status(response, REST.status.SERVICE_UNAVAILABLE);
    REST.set_response_payload(response, NODE_BUSY_TEXT, strlen(NODE_BUSY_TEXT));
    return;
  }

  if (REST.get_method_type(request) == METHOD_GET)
  {

  }
  else if (REST.get_method_type(request) == METHOD_PUT)
  {
    const uint8_t *payload;
    uint8_t len = 0;
    uint32_t num;
    uint32_t boffset;
    uint8_t more;
    uint16_t size;

    len = REST.get_request_payload(request, &payload);
    REST.set_response_status(response, REST.status.CHANGED); // Default return status

    PRINTF("Upload PUT [Len:%u]\n", len);
    if (coap_get_header_block1(request, &num, &more, &size, &boffset))
    {
      PRINTF("Blockwise transer, offset %lu expect more %u\n", boffset, more);
    }
    else
    {
      // If the PUT request is not blockwise there should only be one piece
      boffset = 0;
      size = len;
      more = 0;
    }
    if (boffset == 0)
    {
      PRINTF("Offset 0, init\n");
//      if (state != S_IDLE)
//      {
//        PRINTF("%s\n", RECEIVING_APP_TEXT);
//        REST.set_response_status(response, REST.status.BAD_REQUEST);
//        REST.set_response_payload(response, RECEIVING_APP_TEXT,strlen(RECEIVING_APP_TEXT));
//        return;
//      }
      next_offset = 0;
      // Start timeout timer
      state = S_RECEIVING;
      //ctimer_set(rec_timer, CLOCK_SECOND*60, rec_timeout, 0);
    }
    if (state != S_RECEIVING)
    {
      PRINTF("We are not expecting application data\n");
      REST.set_response_status(response, REST.status.BAD_REQUEST);
      REST.set_response_payload(response, NOT_EXPECTING_DATA_TEXT,strlen(NOT_EXPECTING_DATA_TEXT));
      return;
    }

    if (next_offset != boffset)
    {
      // Not expecting this offset, respond and hope for correct next time.
      PRINTF("******WRONG OFFSET\n");
      return;
    }

    if (!bl_app_data(boffset, payload, len))
    {
      ctimer_stop(rec_timer);
      state = S_IDLE;
      PRINTF("%s\n", UNABLE_TO_WRITE_FLASH_TEXT);
      REST.set_response_status(response, REST.status.BAD_REQUEST);
      REST.set_response_payload(response, UNABLE_TO_WRITE_FLASH_TEXT,strlen(UNABLE_TO_WRITE_FLASH_TEXT));
      return;
    }
    next_offset += REST.get_request_payload(request, &payload);

    if(!more)
    {
      //ctimer_stop(rec_timer);
      state = S_IDLE;
    }
    else
    {
      //ctimer_restart(rec_timer);
    }
  }
}
/*---------------------------------------------------------------------------*/
void rep_start_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
uint8_t rp_buffer[200];
RESOURCE(rep_start, "title=\"Run new application\"", rep_start_handler, NULL, rep_start_handler, NULL);
void
rep_start_handler(void* request, void* response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
  if (state != S_IDLE)
  {
    REST.set_response_status(response, REST.status.SERVICE_UNAVAILABLE);
    REST.set_response_payload(response, NODE_BUSY_TEXT, strlen(NODE_BUSY_TEXT));
    return;
  }
  if (REST.get_method_type(request) == METHOD_GET)
  {
    PRINTF("Start GET\n");
    if (!bl_app_is_empty())
    {
      REST.set_response_payload(response, APP_VERIFIED_TEXT, strlen(APP_VERIFIED_TEXT));
    }
    else
    {
      REST.set_response_payload(response, APP_NOT_VERIFIED_TEXT, strlen(APP_NOT_VERIFIED_TEXT));
    }
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
  }
  else if (REST.get_method_type(request) == METHOD_PUT) {
    const uint8_t *payload;
    uint8_t len = 0;
    PRINTF("Start PUT\n");
    len = REST.get_request_payload(request, &payload);
    if (payload[0] != '1' || len != 1)
    {
      REST.set_response_status(response, REST.status.BAD_REQUEST);
      REST.set_response_payload(response, BAD_INPUT_TEXT, strlen(BAD_INPUT_TEXT));
    }
    state = S_RUN;
    process_poll(&coap_rep_process);
    REST.set_response_status(response, REST.status.CHANGED);
  }
}
/*---------------------------------------------------------------------------*/
void
coap_reprogram_init(void)
{
  process_start(&coap_rep_process, NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(coap_rep_process, ev, data)
{
  PROCESS_BEGIN();

  PRINTF("Start CoAP reprogram process\n");
  // Expecting CoAP to be initialized.
  rest_activate_resource(&rep_upload, "bootloader/upload");
  rest_activate_resource(&rep_start, "bootloader/run");

  while(1) {
    PROCESS_YIELD();
    switch(state)
    {
    case S_RUN:
      PRINTF("CoAP start app\n");
      bootloader_run_app();
      // Will not get here
    }
    state = S_IDLE;
  }
  PROCESS_END();
}
