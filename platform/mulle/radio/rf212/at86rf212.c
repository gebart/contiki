#include "at86rf212.h"
#include "at86rf212_registermap.h"

#include <stddef.h>
#include <stdbool.h>



/* RF212 hardware delay times, from datasheet */
enum
{
  TIME_TO_ENTER_P_ON = 510,                 /**<  Transition time from VCC is applied to P_ON - most favorable case! */
  TIME_P_ON_TO_TRX_OFF = 510,               /**<  Transition time from P_ON to TRX_OFF. */
  TIME_SLEEP_TO_TRX_OFF = 880,              /**<  Transition time from SLEEP to TRX_OFF. */
  TIME_RESET = 6,                           /**<  Time to hold the RST pin low during reset */
  TIME_ED_MEASUREMENT = 140,                /**<  Time it takes to do a ED measurement. */
  TIME_CCA = 140,                           /**<  Time it takes to do a CCA. */
  TIME_PLL_LOCK = 150,                      /**<  Maximum time it should take for the PLL to lock. */
  TIME_FTN_TUNING = 25,                     /**<  Maximum time it should take to do the filter tuning. */
  TIME_NOCLK_TO_WAKE = 6,                   /**<  Transition time from *_NOCLK to being awake. */
  TIME_CMD_FORCE_TRX_OFF = 1,               /**<  Time it takes to execute the FORCE_TRX_OFF command. */
  TIME_TRX_OFF_TO_PLL_ACTIVE = 200,         /**<  Transition time from TRX_OFF to: RX_ON, PLL_ON, TX_ARET_ON and RX_AACK_ON. */
  TIME_STATE_TRANSITION_PLL_ACTIVE = 1,     /**<  Transition time from PLL active state to another. */
};

/* RF212 radio packet */
typedef struct
{
  uint8_t length;       // Length of frame.
  uint8_t data[ 127 ];  // Actual frame data.
  bool crc;             // Flag - did CRC pass for received frame?
} hal_rx_frame_t;

/* Received frames are buffered to rxframe in the interrupt routine */
uint8_t rxframe_head;
uint8_t rxframe_tail;
hal_rx_frame_t rxframe[RF212_RX_BUFFERS];

static bool receive_on = false;
static bool auto_ack = false;
static bool send_on_cca = false;

PROCESS(rf212_process, "RF212 driver");

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rf212_process, ev, data)
{
  int len;
  PROCESS_BEGIN();

  while(1)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_init(void)
{

  delay_us(TIME_TO_ENTER_P_ON);

  /* Initialize Hardware Abstraction Layer */
  hal_init();

  /* Set receive buffers empty and point to the first */
  for(i = 0; i < RF212_RX_BUFFERS; i++)
  {
    rxframe[i].length = 0;
  }
  rxframe_head = 0;
  rxframe_tail = 0;

  /* Do full rf212 Reset */
  hal_set_rst_low();
  LLWU_INHIBIT_LLS();
  hal_set_slptr_low();


  /* On powerup a TIME_RESET delay is needed here, however on some other MCU reset
   * (JTAG, WDT, Brownout) the radio may be sleeping. It can enter an uncertain
   * state (sending wrong hardware FCS for example) unless the full wakeup delay
   * is done.
   * Wake time depends on board capacitance; use 2x the nominal delay for safety.
   * See www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=78725
   */
  delay_us(2 * TIME_SLEEP_TO_TRX_OFF);
  hal_set_rst_high();

  /* Force transition to TRX_OFF */
  hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
  delay_us(TIME_P_ON_TO_TRX_OFF);

  /* Verify that it is a supported version */
  /* Note gcc optimizes this away if DEBUG is not set! */
  /* ATMEGA128RFA1 - version 4, ID 31 */
  uint8_t tvers = hal_register_read(RG_VERSION_NUM);
  uint8_t tmanu = hal_register_read(RG_MAN_ID_0);

  if(tvers != RF212B_VERSION)
  {
    PRINTF("rf212: Unsupported version %u\r\n", tvers);
  }
  if(tmanu != RF212_SUPPORTED_MANUFACTURER_ID)
  {
    PRINTF("rf212: Unsupported manufacturer ID %u\r\n", tmanu);
  }
  PRINTF("rf212: Version %u, ID %u\r\n", tvers, tmanu);

  /* Start the packet receive process */
  process_start(&rf212_process, NULL);

  /* Trick off routine to believe radio is completely on by setting on = true */
  receive_on = true;
  rf212_off();

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_receive_on(void)
{
  if(is_receive_on())
  {
    return 1;
  }
  receive_on = 1;
  on();

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_receive_off(void)
{
  if(!is_receive_on())
  {
    return 0;
  }
  receive_on = false;
  off();

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_cca(void)
{
  uint8_t cca = 0;
  uint8_t radio_was_off = false;

  /* Turn radio on if necessary. If radio is currently busy return busy channel */
  if (!is_receive_on())
  {
    at86rf212_on();
  }
  else if(!is_idle())
  {
    goto exit;
  }

  /* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either (RF231 only) */
  /* Use the current mode. Note triggering a manual CCA is not recommended in extended mode */
  /* hal_subregister_write(SR_CCA_MODE,1); */

  /* Don't allow interrupts! */
  HAL_ENTER_CRITICAL_REGION();
  wait_idle();
  hal_subregister_write(SR_CCA_REQUEST, 1);
  // No need to delay_us, this will wait for CCA to finish
  // TODO(henrik) Break infinite loop after a timeout
  // TODO(henrik) Add mask macros
  while((cca & 0x80) == 0)
  {
    cca = hal_register_read(RG_TRX_STATUS);
  }
  HAL_LEAVE_CRITICAL_REGION();

  exit:
  if(!is_receive_on())
  {
    at86rf212_off();
  }
  if(cca & 0x40)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_prepare(const void *payload, unsigned short payload_len)
{
  int ret = 0;
  uint8_t total_len;
  uint8_t* pbuf;

  if (is_sleeping())
  {
    wakeup();
  }

  /* Wait for any previous operation or state transition to finish */
  /* Do we need to do that? */
  wait_idle();

  // Go into TX state to prevent buffer from being overwritten
  if (send_on_cca)
  {
    radio_set_trx_state(TX_ARET_ON);
  }
  else
  {
    radio_set_trx_state(PLL_ON);
  }

  hal_frame_write(payload, payload_len + AUX_LEN);

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_transmit(unsigned short payload_len)
{
  int txpower;
  uint8_t total_len;
  uint8_t tx_result = RADIO_TX_OK;

  /* Wait for any previous operation or state transition to finish */
  wait_idle();

  /* Toggle the SLP_TR pin to initiate the frame transmission */
  hal_set_slptr_high();
  hal_set_slptr_low();

  /* Get the transmission result */
  if (send_on_cca)
  {
    switch (hal_subregister_read(SR_TRAC_STATUS))
    {
      case 1:
        tx_result = RADIO_TX_OK;
        break;
      case 3:
        tx_result = RADIO_TX_COLLISION;
        break;
      case 5:
        tx_result = RADIO_TX_NOACK;
        break;
      default:
        tx_result = RADIO_TX_ERR;
    }
  }
  if(is_receive_on())
  {
    on();
  }
  else
  {
    off();
  }

  return tx_result;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_send(const void *payload, unsigned short payload_len)
{
  int ret;
  if ((ret = at86rf212_prepare(payload, payload_len)) != 0)
  {
    return ret;
  }

  return at86rf212_transmit(payload_len);
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_pending_packet()
{
  return rxframe[rxframe_tail].length > 0;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_receiving_packet(void)
{
  uint8_t radio_state;
  if(!sleeping())
  {
    radio_state = hal_subregister_read(SR_TRX_STATUS);
    return (radio_state == BUSY_RX) || (radio_state == BUSY_RX_AACK);
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_read(void *buf, unsigned short bufsize)
{
  uint8_t len;
  uint8_t *framep;

  /*
   * Read could be allowed even if rx is off or radio is sleeping because
   * packets are buffered.
   */
  if(is_sleeping())
  {
    return 0;
  }
  if(!is_receive_on())
  {
    return 0;
  }

  /* The length includes the twp-byte checksum but not the LQI byte */
  len = rxframe[rxframe_tail].length;
  if(len == 0)
  {
    flushrx();
    return 0;
  }

  if(len <= AUX_LEN)
  {
    flushrx();
    return 0;
  }

  if(len - AUX_LEN > bufsize)
  {
    /* Packet doesnt fit in provided buffer */
    flushrx();
    return 0;
  }
  /* Transfer the frame, stripping the footer, but copying the checksum */
  framep = &(rxframe[rxframe_tail].data[0]);
  memcpy(buf, framep, len - AUX_LEN + CHECKSUM_LEN);

  /* Prepare to receive another packet */
  flushrx();

  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, 50);
  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, 50);

  /*
   * Here return just the data length. The checksum is
   * however still in the buffer for packet sniffing
   */
  return len - AUX_LEN;
}
/*---------------------------------------------------------------------------*/
static void
at86rf212_poll()
{
   process_poll(&rf212_process);
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_get_value(radio_param_t param, radio_value_t *value)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_set_value(radio_param_t param, radio_value_t value)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_get_object(radio_param_t param, void *dest, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_set_object(radio_param_t param, const void *src, size_t size)
{
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static void
on()
{
  /* If radio is sleeping turn it on */
  if(is_sleeping())
  {
    wakeup();
  }

  if (auto_ack)
  {
    radio_set_trx_state(RX_AACK_ON);
  }
  else
  {
    radio_set_trx_state(RX_ON);
  }
  wait_idle();
}
/*---------------------------------------------------------------------------*/
static void
off()
{
  /* Should we wait for packet reception to finish? */
  /* Should we wait for packet transmission to finish? */
  /* Wait for any transmission to end */
  wait_idle();

  /* Force the device into TRX_OFF. */
  radio_reset_state_machine();

  /* Sleep Radio */
  if(!is_sleeping())
  {
    LLWU_UNINHIBIT_LLS();
    hal_set_slptr_high();
  }
}
/*---------------------------------------------------------------------------*/
static bool
is_receive_on()
{
  return receive_on;
}
/*---------------------------------------------------------------------------*/
static bool
is_sleeping()
{
  return hal_get_slptr();
}
/*---------------------------------------------------------------------------*/
static void
wakeup()
{
  /* Inhibit the lowest sleep modes since we can not wake the MCU using the
   * radio IRQ pin on Mulle. (Platform board errata)
   */
  LLWU_INHIBIT_LLS();
  hal_disable_trx_interrupt();
  hal_set_slptr_low();
  while(hal_get_irq() == 0) {}
  hal_register_read(RG_IRQ_STATUS); // Clear interrupts
  hal_enable_trx_interrupt();
}
/*---------------------------------------------------------------------------*/
static bool
is_idle(void)
{
  uint8_t radio_state;
  if(is_sleeping())
  {
    return 1;
  }
  else
  {
    radio_state = hal_subregister_read(SR_TRX_STATUS);
    if(radio_state != BUSY_TX_ARET &&
        radio_state != BUSY_RX_AACK &&
        radio_state != STATE_TRANSITION &&
        radio_state != BUSY_RX &&
        radio_state != BUSY_TX)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
wait_idle(void)
{
  int i;
  for(i = 0; i < 10000; i++)
  {  /* to avoid potential hangs */
    if(is_idle())
    {
      break;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
flushrx(void)
{
  /* Clear the length field to allow buffering of the next packet */
  rxframe[rxframe_tail].length=0;
  rxframe_tail++;
  if (rxframe_tail >= RF212_RX_BUFFERS)
  {
    rxframe_tail=0;
  }
  /* If another packet has been buffered, schedule another receive poll */
  if (at86rf212_pending_packet())
  {
    at86rf212_poll();
  }
}
/*---------------------------------------------------------------------------*/
const struct radio_driver rf212_driver =
    {
        at86rf212_init,
        at86rf212_prepare,
        at86rf212_transmit,
        at86rf212_send,
        at86rf212_read,
        at86rf212_cca,
        at86rf212_receiving_packet,
        at86rf212_pending_packet,
        at86rf212_receive_on,
        at86rf212_receive_off,
        at86rf212_get_value,
        at86rf212_set_value,
        at86rf212_get_object,
        at86rf212_set_object
    };
