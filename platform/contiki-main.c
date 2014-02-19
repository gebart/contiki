#include "contiki.h"
#include "sys/autostart.h"

#include "MK60N512VMD100.h"
#include "stdio.h"
#include "uart.h"

/* C entry point (after startup code has executed) */
int main(void)
{
  uart_init();
  /*
   * Initialize Contiki and our processes.
   */
  process_init();
  process_start(&etimer_process, NULL);

  ctimer_init();

  clock_init();

  autostart_start(autostart_processes);

  while (1)
  {
    process_run();
  }
}
