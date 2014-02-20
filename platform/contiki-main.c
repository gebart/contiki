#include "stdio.h"

#include "contiki.h"
#include "sys/autostart.h"

#include "MK60N512VMD100.h"
#include "uart.h"
#include "udelay.h"
#include "init-net.h"

/* C entry point (after startup code has executed) */
int main(void)
{
  uart_init();
  udelay_init();
  printf("Booted\n");
  /*
   * Initialize Contiki and our processes.
   */
  process_init();
  process_start(&etimer_process, NULL);

  ctimer_init();

  clock_init();
  init_net();

  autostart_start(autostart_processes);

  while (1)
  {
    process_run();
  }
}
