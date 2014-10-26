/*
 * Configuration for SPI0 bus.
 */

/*
 * Onboard devices connected:
 *  AT86RF212: max sck frequency 7.5 MHz (async) 8000000 (sync), CPOL = 0, CPHA = 0, but seems to work with CPOL=1, CPHA=1 as well.
 *  LIS3DH: max sck frequency 10 MHz, CPOL = 1, CPHA = 1
 *  M25P16: max sck frequency 25 MHz, (CPOL=0, CPHA=0) or (CPOL=1, CPHA=1)
 *  FM25L04B: max sck frequency 20 MHz, (CPOL=0, CPHA=0) or (CPOL=1, CPHA=1)
 *
 * The AT86RF212 is allocated to CTAR0, the rest at CTAR1.
 * FIXME: The memories could be driven faster.
 */

#include "spi_arch.h"

spi_config_t spi0_conf[NUM_CTAR] = {
  { .sck_freq = 7500000, .frame_size = 8, .cpol = 0, .cpha = 0},
  { .sck_freq = 10000000, .frame_size = 8, .cpol = 1, .cpha = 1}
  };

/* SPI1 is only used by expansion boards. */
/*
spi_config_t spi1_conf[NUM_CTAR] = {
  { .sck_freq = 1000000, .frame_size = 8, .cpol = 0, .cpha = 0},
  { .sck_freq = 10000000, .frame_size = 8, .cpol = 1, .cpha = 1}
  };
*/
