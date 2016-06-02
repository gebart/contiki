#include "K60.h"

#include "contiki-conf.h"
#include "board.h"
#include "periph/gpio.h"
#include "dev/leds.h"
/*---------------------------------------------------------------------------*/
void
leds_arch_init(void)
{
  gpio_init(LED_RED_GPIO, GPIO_DIR_OUT, GPIO_NOPULL);
  gpio_init(LED_YELLOW_GPIO, GPIO_DIR_OUT, GPIO_NOPULL);
  gpio_init(LED_GREEN_GPIO, GPIO_DIR_OUT, GPIO_NOPULL);
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  unsigned char leds = 0;
  if (gpio_read(LED_RED_GPIO) == 1) {
    leds |= LEDS_RED;
  }
  if (gpio_read(LED_YELLOW_GPIO) == 1) {
    leds |= LEDS_YELLOW;
  }
  if (gpio_read(LED_GREEN_GPIO) == 1) {
    leds |= LEDS_GREEN;
  }
  return leds;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  #if CONFIG_LOW_POWER_LEDS
  #else
  unsigned char value;
  value = (((leds & LEDS_RED) != 0) ? 1 : 0);
  gpio_write(LED_RED_GPIO, value);
  value = (((leds & LEDS_YELLOW) != 0) ? 1 : 0);
  gpio_write(LED_YELLOW_GPIO, value);
  value = (((leds & LEDS_GREEN) != 0) ? 1 : 0);
  gpio_write(LED_GREEN_GPIO, value);
  #endif
}
/*---------------------------------------------------------------------------*/
