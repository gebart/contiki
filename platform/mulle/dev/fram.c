#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "K60.h"
#include "spi-k60.h"

#include "fram.h"

#include "clock.h"
#include "udelay.h"

/* ************************************************************************** */
/* #define FRAM_DEBUG */
#ifdef FRAM_DEBUG
#include <stdio.h>
#define DEBUG(...) printf(__VA_ARGS__)
#else
#define DEBUG(...) (void)(0);
#endif

/* ************************************************************************** */


fram_error_t
fram_init(void)
{
  uint32_t status;
  uint32_t jedec_id;
  int i;

  /* Wait a while for memories to start */
  /* Data sheet for M25P16 says this should be around 10 ms max, but this may
   * also depend on how much decoupling is used for the power circuit. */
  /* TODO(henrik) Change this to more exact times. */
  for(i = 0; i < 200; ++i) {
    udelay(1000);
  }

  spi_acquire_bus(FRAM_SPI_NUM);
   //status = cmd_rdsr(i);
  spi_release_bus(FRAM_SPI_NUM);
/*
    if(status) {
      DEBUG("Error: Status of flash 0 is non-zero (0x%02x).\n", status);
      return E_FLASH_INVALID_STATUS;
    }
    spi_acquire_bus(FLASH_SPI_NUM);
    jedec_id = cmd_rdid(i);
    spi_release_bus(FLASH_SPI_NUM);
    if(jedec_id != expected_jedec_id.u32) {
      DEBUG("Flash0: Invalid JEDEC-ID: 0x%08x\n", jedec_id);
      return E_FLASH_UNKNOWN;
    }
  }*/

  return E_FRAM_OK;
}
/* ************************************************************************** */


