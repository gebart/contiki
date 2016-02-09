
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include "dev/slip.h"

#include "uart.h"
#include "periph/port.h"
#include "config-board.h"
#include "llwu.h"

static int slip_fd = -1;

/*---------------------------------------------------------------------------*/
void
slip_arch_init(unsigned long ubr)
{
  /* Turn on the hardware pins */
  port_init(BOARD_SLIP_UART_RX_GPIO, GPIO_NOPULL, BOARD_SLIP_UART_RX_AF);
  port_init(BOARD_SLIP_UART_TX_GPIO, GPIO_NOPULL, BOARD_SLIP_UART_TX_AF);

  /* (Re-)initialize the UART module */
  uart_init(BOARD_SLIP_UART_NUM, 0, ubr);

  int fd;
  struct stat st;
  fd = open(BOARD_SLIP_UART_NAME, O_RDWR, 0);
  if (fd < 0) {
    /* Failed to open device */
    return;
  }

  if (fstat(fd, &st) != 0) {
    /* fstat failed to check status of the open fd */
    return;
  }
  if (!S_ISCHR(st.st_mode)) {
    /* Not a character device */
    return;
  }
  if(!isatty(fd)) {
    /* Not a tty device */
    return;
  }

  slip_fd = fd;
  uart_set_rx_callback(BOARD_SLIP_UART_NUM, slip_input_byte);
  uart_enable_rx_interrupt(BOARD_SLIP_UART_NUM);
  /* Don't allow LLS since it will disable the UART module clock, which prevents
   * any incoming bytes from being detected. */
  LLWU_INHIBIT_LLS();
}
/*---------------------------------------------------------------------------*/
void
slip_arch_writeb(unsigned char c)
{
  write(slip_fd, &c, 1);
}
