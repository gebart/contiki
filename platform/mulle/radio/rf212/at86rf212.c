#include "at86rf212.h"
#include "at86rf212_hal.h"
#include "at86rf212_registermap.h"

#include "radio.h"
#include "rtimer.h"
#include "packetbuf.h"
#include "netstack.h"
#include "leds.h"

#include <string.h>

/* TODO(henrik)Maybe turn off/on interrupts and disable enable LLWU through that routine on
 * the hal layer.
 */
#include "llwu.h"

#include <stddef.h>
#include <stdbool.h>

#include <stdio.h>
/* Debugging*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF printf
#else
#define PRINTF(...)
#endif

/* Interrupt sources */
enum
{
  HAL_BAT_LOW_MASK =      (0x80),   /**< Mask for the BAT_LOW interrupt. */
  HAL_TRX_UR_MASK =       (0x40),   /**< Mask for the TRX_UR interrupt. */
  HAL_TRX_END_MASK =      (0x08),   /**< Mask for the TRX_END interrupt. */
  HAL_RX_START_MASK =     (0x04),   /**< Mask for the RX_START interrupt. */
  HAL_PLL_UNLOCK_MASK =   (0x02),   /**< Mask for the PLL_UNLOCK interrupt. */
  HAL_PLL_LOCK_MASK =     (0x01),   /**< Mask for the PLL_LOCK interrupt. */
};

/* RF212 does not support RX_START interrupts in extended mode, but it seems harmless to always enable it. */
/* In non-extended mode this allows RX_START to sample the RF rssi at the end of the preamble */
#define RF212_SUPPORTED_INTERRUPT_MASK          (0x0C)    /* disable bat low, trx underrun, pll lock/unlock */


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

/*
 * Radio driver is expected to go into RX state after transmit. RF212 uses same
 * buffer for TX and RX so the packet must be saved in ram on the MCU so that
 * a TX packet does not get overwritten between sends.
 *
 * Defining TX_BUF sets the radio into RX after transmit, not defining TX_BUF
 * leaves radio in TX after a transmitt and keeps the packet in the radio buffer.
 */
#define TX_BUF 1

/* Received frames are buffered to rxframe in the interrupt routine */
uint8_t rxframe_head;
uint8_t rxframe_tail;
hal_rx_frame_t rxframe[RF212_RX_BUFFERS];

#ifdef TX_BUF
static uint8_t tx_buf[127];
static int tx_len;
#endif

static bool receive_on = false;
static bool frame_filtering = false;
static bool auto_ack = false;
static bool poll_mode = false;
static uint8_t csma_retries = 0;
static uint8_t channel = 0;
static bool send_on_cca = false;
rtimer_clock_t rf212_sfd_start_time;

static int at86rf212_read(void *buf, unsigned short bufsize);

static void on(void);
static void off(void);
static bool is_receive_on(void);
static bool is_sleeping(void);
static void wakeup(void);
static bool is_idle(void);
static void wait_idle(void);
static void flushrx(void);
static uint8_t get_trx_state(void);
static void reset_state_machine(void);
static radio_status_t radio_set_trx_state(uint8_t new_state);
static void set_auto_ack(uint8_t enable);
static void set_frame_filtering(uint8_t enable);
static void set_poll_mode(uint8_t enable);
static bool set_cca_retries(uint8_t num);
static void set_send_on_cca(bool enable);
static bool set_tx_retries(uint8_t num);
static uint8_t get_channel(void);
static void set_channel(uint8_t c);
static void at86rf212_rx_event(void);

static volatile rtimer_clock_t poll_timestamp = 0;

PROCESS(rf212_process, "RF212 driver");
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(rf212_process, ev, data)
{
  PROCESS_BEGIN();

  while(1)
  {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL && !poll_mode);
#ifdef WITH_SLIP
    leds_toggle(LEDS_RED);
#endif
#ifdef RF212_IRQ_POLL
    if (at86rf212_interrupt(poll_timestamp))
    {
#endif
      at86rf212_rx_event();
#ifdef RF212_IRQ_POLL
    }
#endif
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/* Called from rf212_process when the radio has signaled that a new frame has arrived. */
static void
at86rf212_rx_event(void)
{
  int len;
  packetbuf_clear();
  len = at86rf212_read(packetbuf_dataptr(), PACKETBUF_SIZE);

  if(len > 0)
  {
    packetbuf_set_datalen(len);
    NETSTACK_RDC.input();
  }
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_init(void)
{
  int i;
  PRINTF("%s\n",__FUNCTION__);

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

  // Enable PLL_LOCK and CCA_ED_DONE interrupts.
  // CCA_ED_DONE is also triggered when going from SLEEP -> TRX_OFF
  // PLL_LOCK could also be done with IRQ_MASK_MODE (register 0x04) but SLEEP > TRX_OFF can not.
  // CCA_ED_DONE could be disabled after wakeup to prevent interrupts when doing CCA.
  hal_register_write(RG_IRQ_MASK, RF212_SUPPORTED_INTERRUPT_MASK|0x10|0x1);

  /* Default settings */
  set_auto_ack(RF212_HARDWARE_ACK);
  set_cca_retries(RF212_CCA_RETRIES);
  set_send_on_cca(RF212_SEND_ON_CCA);
  set_frame_filtering(RF212_FRAME_FILTERING);
  set_tx_retries(RF212_AUTORETRIES);
  #ifdef FRAME802154_CONF_VERSION
  /* Set frame filtering on frame version in transceiver to accept frame versions 0, 1, 2 */
    hal_subregister_write(SR_AACK_FVN_MODE, FRAME802154_CONF_VERSION);
  #endif
  hal_subregister_write(RG_TRX_CTRL_2, 0x3F, 0, RF212_PHY_MODE);

  /* Dont know if this really makes any difference */
  /* CCA Mode Mode 1=Energy above threshold  2=Carrier sense only  3=Both 0=Either */
  hal_subregister_write(SR_CCA_MODE, 0); // Carrier sense or above threshold

  /* Start the packet receive process */
  process_start(&rf212_process, NULL);

  off();

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_receive_on(void)
{
  PRINTF("%s\n",__FUNCTION__);
  receive_on = 1;
  on();

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_receive_off(void)
{
  PRINTF("%s\n",__FUNCTION__);
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
  PRINTF("%s\n",__FUNCTION__);
  uint8_t cca = 0;

  /* Turn radio on if necessary. If radio is currently busy return busy channel */
  /* A manual CCA check should not be done in extended operation mode */
  if (!is_receive_on())
  {
    /* If radio is sleeping turn it on */
    if(is_sleeping())
    {
      wakeup();
    }
  }
  else if(!is_idle())
  {
    goto exit;
  }

  /* Don't allow interrupts! */
  HAL_ENTER_CRITICAL_REGION();

  // Go to RX_ON state
  hal_subregister_write(0x15, 0x80, 7, 1); // Disable frame reception
  radio_set_trx_state(RX_ON);

  wait_idle();

  hal_subregister_write(SR_CCA_REQUEST, 1);
  // No need to delay_us, this will wait for CCA to finish
  // TODO(henrik) Break infinite loop after a timeout
  // TODO(henrik) Add mask macros
  while((cca & 0x80) == 0)
  {
    cca = hal_register_read(RG_TRX_STATUS);
  }
  hal_register_read(RG_IRQ_STATUS); // Clear interrupts
  radio_set_trx_state(RX_AACK_ON);
  hal_subregister_write(0x15, 0x80, 7, 0); // Enable frame reception
  HAL_LEAVE_CRITICAL_REGION();

  exit:
  if(!is_receive_on())
  {
    at86rf212_receive_off();
  }
  if(cca & 0x40)
  {
    /* Idle */
    return 1;
  }
  else
  {
    /* Busy */
    return 0;
  }
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_prepare(const void *payload, unsigned short payload_len)
{
  PRINTF("%s\n",__FUNCTION__);

#ifdef TX_BUF
  memcpy(tx_buf, payload, payload_len);
  tx_len = payload_len;
#else // Buffer directly on radio
  /* Wait for any previous operation or state transition to finish */
  /* Do we need to do that? */
  wait_idle();

  // Go into TX state to prevent buffer from being overwritten
  radio_set_trx_state(TX_ARET_ON);

  hal_frame_write(payload, payload_len + AUX_LEN);
#endif
  return RADIO_RESULT_OK;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_transmit(unsigned short payload_len)
{
  uint8_t tx_result = RADIO_TX_OK;

  if (is_sleeping())
  {
    wakeup();
  }

  /* Wait for any previous operation or state transition to finish */
  wait_idle();

#ifdef TX_BUF
  // Go into TX state to prevent buffer from being overwritten
  radio_set_trx_state(TX_ARET_ON);

  hal_frame_write(tx_buf, tx_len + AUX_LEN);
#endif
  set_send_on_cca(send_on_cca);

  /* Toggle the SLP_TR pin to initiate the frame transmission */
  hal_set_slptr_high();
  hal_set_slptr_low();

  wait_idle();

  /* Get the transmission result */
  switch (hal_subregister_read(SR_TRAC_STATUS))
  {
    case 0:
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

  // CSMA retries == 7 means no CSMA
  if (hal_subregister_read(SR_MAX_CSMA_RETRIES) == 7)
  {
    tx_result = RADIO_TX_OK;
  }

#ifdef TX_BUF
  at86rf212_receive_on();
#endif

  PRINTF("%s: %d %d\n",__FUNCTION__, payload_len, tx_result);
  return tx_result;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_send(const void *payload, unsigned short payload_len)
{
  PRINTF("%s: %d\n",__FUNCTION__, payload_len);
  int ret;
  if ((ret = at86rf212_prepare(payload, payload_len)) != RADIO_RESULT_OK)
  {
    return ret;
  }
  ret = at86rf212_transmit(payload_len);
#ifndef TX_BUF
  if(is_receive_on())
  {
    on();
  }
  else
  {
    off();
  }
#endif
  return ret;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_pending_packet(void)
{
  PRINTF("%s: %d\n",__FUNCTION__, rxframe[rxframe_tail].length > 0);
  return rxframe[rxframe_tail].length > 0;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_receiving_packet(void)
{
  uint8_t radio_state;
  if(!is_sleeping())
  {
    radio_state = hal_subregister_read(SR_TRX_STATUS);
    PRINTF("%s: %d\n",__FUNCTION__, (radio_state == BUSY_RX) || (radio_state == BUSY_RX_AACK));
    return (radio_state == BUSY_RX) || (radio_state == BUSY_RX_AACK);
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
at86rf212_read(void *buf, unsigned short bufsize)
{
  PRINTF("%s %d\n",__FUNCTION__, rxframe[rxframe_tail].length);
  uint8_t len;
  uint8_t *framep;

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

  packetbuf_set_attr(PACKETBUF_ATTR_RSSI, rxframe[rxframe_tail].rssi);

  /* Prepare to receive another packet */
  flushrx();

  packetbuf_set_attr(PACKETBUF_ATTR_LINK_QUALITY, 50);

  /*
   * Here return just the data length. The checksum is
   * however still in the buffer for packet sniffing
   */

  PRINTF("%s: %u bytes\n", __FUNCTION__, len - AUX_LEN);
  return len - AUX_LEN;
}
/*---------------------------------------------------------------------------*/
void
at86rf212_poll(void)
{
  PRINTF("%s\n",__FUNCTION__);
#ifdef RF212_IRQ_POLL
  poll_timestamp = RTIMER_NOW();
#endif
  process_poll(&rf212_process);
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_get_value(radio_param_t param, radio_value_t *value)
{
  PRINTF("%s\n",__FUNCTION__);
  if(value == NULL) {
    return RADIO_RESULT_INVALID_VALUE;
  }

  switch(param) {
    case RADIO_PARAM_RSSI:
      //TODO(henrik) FIX
      *value = 40; //rf230_get_raw_rssi();
      return RADIO_RESULT_OK;

    case RADIO_PARAM_LAST_LINK_QUALITY:
      //TODO(henrik) FIX
      *value = 40; //rf230_last_lqi;
      return RADIO_RESULT_OK;

    case RADIO_PARAM_LAST_RSSI:
      //TODO(henrik) FIX
      *value = 40; //rf230_last_rssi;
      return RADIO_RESULT_OK;

    case RADIO_PARAM_RX_MODE:
      *value = 0;
      // TODO(henrik): Masks for positions.
      if(frame_filtering) {
        *value |= RADIO_RX_MODE_ADDRESS_FILTER;
      }
      if (auto_ack) {
        *value |= RADIO_RX_MODE_AUTOACK;
      }
      if(poll_mode) {
        *value |= RADIO_RX_MODE_POLL_MODE;
      }
      return RADIO_RESULT_OK;

    case RADIO_CONST_CHANNEL_MIN:
      *value = 0;
      return RADIO_RESULT_OK;

    case RADIO_CONST_CHANNEL_MAX:
      *value = 10;
      return RADIO_RESULT_OK;

    case RADIO_PARAM_CHANNEL:
      *value = get_channel();
      return RADIO_RESULT_OK;

    case RADIO_PARAM_TX_MODE:
      *value = 0;
      if(send_on_cca)
      {
        *value |= RADIO_TX_MODE_SEND_ON_CCA;
      }
      return RADIO_RESULT_OK;

    default:
      return RADIO_RESULT_NOT_SUPPORTED;
  }
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_set_value(radio_param_t param, radio_value_t value)
{
  PRINTF("%s\n",__FUNCTION__);
  switch(param)
  {
    case RADIO_PARAM_RX_MODE:
      if(value & ~(RADIO_RX_MODE_ADDRESS_FILTER |
          RADIO_RX_MODE_AUTOACK | RADIO_RX_MODE_POLL_MODE))
      {
        return RADIO_RESULT_INVALID_VALUE;
      }
      frame_filtering = (value & RADIO_RX_MODE_ADDRESS_FILTER) != 0;
      auto_ack = (value & RADIO_RX_MODE_AUTOACK) != 0;
      poll_mode = (value & RADIO_RX_MODE_POLL_MODE) != 0;

      set_frame_filtering(frame_filtering);
      set_auto_ack(auto_ack);
      set_poll_mode(poll_mode);
      return RADIO_RESULT_OK;

    case RADIO_PARAM_CHANNEL:
      channel = value;
      set_channel(value);
      return RADIO_RESULT_OK;

    case RADIO_PARAM_TX_MODE:
      if(value & ~(RADIO_TX_MODE_SEND_ON_CCA))
      {
        return RADIO_RESULT_INVALID_VALUE;
      }
      set_send_on_cca((value & RADIO_TX_MODE_SEND_ON_CCA) != 0);
      return RADIO_RESULT_OK;
  }

  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_get_object(radio_param_t param, void *dest, size_t size)
{
  PRINTF("%s\n",__FUNCTION__);
  if(param == RADIO_PARAM_LAST_PACKET_TIMESTAMP)
  {
    if(size != sizeof(rtimer_clock_t) || !dest) {
      return RADIO_RESULT_INVALID_VALUE;
    }
    *(rtimer_clock_t*)dest = rf212_sfd_start_time;
    return RADIO_RESULT_OK;
  }
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static radio_result_t
at86rf212_set_object(radio_param_t param, const void *src, size_t size)
{
  PRINTF("%s\n",__FUNCTION__);
  return RADIO_RESULT_NOT_SUPPORTED;
}
/*---------------------------------------------------------------------------*/
static void
on(void)
{
  /* If radio is sleeping turn it on */
  if(is_sleeping())
  {
    wakeup();
  }

  set_auto_ack(auto_ack);
  set_frame_filtering(frame_filtering);
  radio_set_trx_state(RX_AACK_ON);
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

  /*
   * Radio needs high priority on interrupt, if it runs with a lower priority
   * than the routine calling off() a reception will be broken.
   * A workaround could be to save the message to the buffer here.
   */

  /* Force the device into TRX_OFF. */
  reset_state_machine();

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
  set_channel(channel);
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
/*----------------------------------------------------------------------------*/
/** \brief  This function will change the current state of the radio
 *          transceiver's internal state machine.
 *
 *  \param     new_state        Here is a list of possible states:
 *             - RX_ON        Requested transition to RX_ON state.
 *             - TRX_OFF      Requested transition to TRX_OFF state.
 *             - PLL_ON       Requested transition to PLL_ON state.
 *             - RX_AACK_ON   Requested transition to RX_AACK_ON state.
 *             - TX_ARET_ON   Requested transition to TX_ARET_ON state.
 *
 *  \retval    RADIO_SUCCESS          Requested state transition completed
 *                                  successfully.
 *  \retval    RADIO_INVALID_ARGUMENT Supplied function parameter out of bounds.
 *  \retval    RADIO_WRONG_STATE      Illegal state to do transition from.
 *  \retval    RADIO_BUSY_STATE       The radio transceiver is busy.
 *  \retval    RADIO_TIMED_OUT        The state transition could not be completed
 *                                  within resonable time.
 */
static radio_status_t
radio_set_trx_state(uint8_t new_state)
{
  uint8_t original_state;

  /* Check function parameter and current state of the radio transceiver. */
  switch (new_state)
  {
    case TRX_OFF:
    case RX_ON:
    case PLL_ON:
    case RX_AACK_ON:
    case TX_ARET_ON:
      /* OK */
      break;
    default:
      /* Invalid new state */
      return RADIO_INVALID_ARGUMENT;
  }

  if(is_sleeping())
  {
    return RADIO_WRONG_STATE;
  }

  /* Wait for radio to finish previous operation */
  wait_idle();

  original_state = get_trx_state();

  PRINTF("%s: %d->%d\n",__FUNCTION__, original_state, new_state);

  if(new_state == original_state)
  {
    return RADIO_SUCCESS;
  }

  /*
   *  At this point it is clear that the requested new_state is:
   * TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON or TX_ARET_ON.
   */

  /*
   *  The radio transceiver can be in one of the following states:
   * TRX_OFF, RX_ON, PLL_ON, RX_AACK_ON, TX_ARET_ON.
   */
  if(new_state == TRX_OFF)
  {
    reset_state_machine();     /* Go to TRX_OFF from any state. */
  }
  else
  {
    /*
     * It is not allowed to go from RX_AACK_ON or TX_AACK_ON and directly to
     * TX_AACK_ON or RX_AACK_ON respectively. Need to go via RX_ON or PLL_ON.
     */
    if( (new_state == TX_ARET_ON) && (original_state == RX_AACK_ON) )
    {
      /*
       * First do intermediate state transition to PLL_ON, then to TX_ARET_ON.
       * The final state transition to TX_ARET_ON is handled after the if-else if.
       */
      hal_subregister_write(SR_TRX_CMD, PLL_ON);
      delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
    }
    else if( (new_state == RX_AACK_ON) && (original_state == TX_ARET_ON) )
    {
      /*
       * First do intermediate state transition to RX_ON, then to RX_AACK_ON.
       * The final state transition to RX_AACK_ON is handled after the if-else if.
       */
      hal_subregister_write(SR_TRX_CMD, RX_ON);
      delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
    }

    /* Any other state transition can be done directly. */
    hal_subregister_write(SR_TRX_CMD, new_state);

    /*
     * When the PLL is active most states can be reached in 1us. However, from
     * TRX_OFF the PLL needs time to activate.
     */
    if(original_state == TRX_OFF)
    {
      hal_disable_trx_interrupt();
      while(hal_get_irq() == 0) {}
      hal_register_read(RG_IRQ_STATUS); // Clear interrupts
      hal_enable_trx_interrupt();
    }
    else
    {
      delay_us(TIME_STATE_TRANSITION_PLL_ACTIVE);
    }
  }

  /* Verify state transition. */
  radio_status_t set_state_status = RADIO_TIMED_OUT;

  if(get_trx_state() == new_state)
  {
    set_state_status = RADIO_SUCCESS;
  }

  return set_state_status;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function return the Radio Transceivers current state.
 *
 *  \retval     P_ON               When the external supply voltage (VDD) is
 *                                 first supplied to the transceiver IC, the
 *                                 system is in the P_ON (Poweron) mode.
 *  \retval     BUSY_RX            The radio transceiver is busy receiving a
 *                                 frame.
 *  \retval     BUSY_TX            The radio transceiver is busy transmitting a
 *                                 frame.
 *  \retval     RX_ON              The RX_ON mode enables the analog and digital
 *                                 receiver blocks and the PLL frequency
 *                                 synthesizer.
 *  \retval     TRX_OFF            In this mode, the SPI module and crystal
 *                                 oscillator are active.
 *  \retval     PLL_ON             Entering the PLL_ON mode from TRX_OFF will
 *                                 first enable the analog voltage regulator. The
 *                                 transceiver is ready to transmit a frame.
 *  \retval     BUSY_RX_AACK       The radio was in RX_AACK_ON mode and received
 *                                 the Start of Frame Delimiter (SFD). State
 *                                 transition to BUSY_RX_AACK is done if the SFD
 *                                 is valid.
 *  \retval     BUSY_TX_ARET       The radio transceiver is busy handling the
 *                                 auto retry mechanism.
 *  \retval     RX_AACK_ON         The auto acknowledge mode of the radio is
 *                                 enabled and it is waiting for an incomming
 *                                 frame.
 *  \retval     TX_ARET_ON         The auto retry mechanism is enabled and the
 *                                 radio transceiver is waiting for the user to
 *                                 send the TX_START command.
 *  \retval     RX_ON_NOCLK        The radio transceiver is listening for
 *                                 incomming frames, but the CLKM is disabled so
 *                                 that the controller could be sleeping.
 *                                 However, this is only true if the controller
 *                                 is run from the clock output of the radio.
 *  \retval     RX_AACK_ON_NOCLK   Same as the RX_ON_NOCLK state, but with the
 *                                 auto acknowledge module turned on.
 *  \retval     BUSY_RX_AACK_NOCLK Same as BUSY_RX_AACK, but the controller
 *                                 could be sleeping since the CLKM pin is
 *                                 disabled.
 *  \retval     STATE_TRANSITION   The radio transceiver's state machine is in
 *                                 transition between two states.
 */
/* static uint8_t */
static uint8_t
get_trx_state(void)
{
  return hal_subregister_read(SR_TRX_STATUS);
}
/*----------------------------------------------------------------------------*/
/** \brief  This function will reset the state machine (to TRX_OFF) from any of
 *          its states.
 */
static void
reset_state_machine(void)
{
  if (is_sleeping())
  {
    wakeup();
  }
  delay_us(TIME_NOCLK_TO_WAKE);
  hal_subregister_write(SR_TRX_CMD, CMD_FORCE_TRX_OFF);
  delay_us(TIME_CMD_FORCE_TRX_OFF);
}
/*---------------------------------------------------------------------------*/
/* Set or unset frame autoack */
static void
set_auto_ack(uint8_t enable)
{
  if (is_sleeping())
  {
    return;
  }
  HAL_ENTER_CRITICAL_REGION();
  auto_ack = enable;
  hal_subregister_write(RG_CSMA_SEED_1, 0x10, 4, !enable);
  HAL_LEAVE_CRITICAL_REGION();
}
/*---------------------------------------------------------------------------*/
/* Set or unset frame filtering */
static void
set_frame_filtering(uint8_t enable)
{
  if (is_sleeping())
  {
    return;
  }
  HAL_ENTER_CRITICAL_REGION();
  frame_filtering = enable;
  hal_subregister_write(RG_XAH_CTRL_1, 0x2, 1, !enable);
  HAL_LEAVE_CRITICAL_REGION();
}
/*---------------------------------------------------------------------------*/
/* Enable or disable packet signaling */
static void
set_poll_mode(uint8_t enable)
{
  if (is_sleeping())
  {
    return;
  }
  HAL_ENTER_CRITICAL_REGION();
  poll_mode = enable;
  HAL_LEAVE_CRITICAL_REGION();
}
/*---------------------------------------------------------------------------*/
/* Sets the number CCA retries in TX_ARET state.
 * CCA must also be enabled with set_send_on_cca.
 */
static bool
set_cca_retries(uint8_t num)
{
  if (num > 5)
  {
    /* Max value is 5 */
    return false;
  }
  csma_retries = num;
  if (hal_subregister_read(SR_MAX_CSMA_RETRIES) != 7)
  {
    hal_subregister_write(SR_MAX_CSMA_RETRIES, num);
  }
  return true;
}
/*---------------------------------------------------------------------------*/
/* Enables or disables CCA before sending */
static void
set_send_on_cca(bool enable)
{
  if (is_sleeping())
  {
    return;
  }
  send_on_cca = enable;
  if (enable)
  {
    hal_subregister_write(SR_MAX_CSMA_RETRIES, csma_retries);
  }
  else
  {
    /* CSMA retries value of 7 disables CSMA */
    hal_subregister_write(SR_MAX_CSMA_RETRIES, 7);
  }
}
/*---------------------------------------------------------------------------*/
static bool
set_tx_retries(uint8_t num)
{
  if (num > 15)
  {
    return false;
  }
  hal_subregister_write(SR_MAX_FRAME_RETRIES, num);
  return true;
}
/*---------------------------------------------------------------------------*/
/* This functionality could be moved to set_value/set_object */
void
at86rf212_set_pan_addr(unsigned pan,
                       unsigned addr,
                       const uint8_t ieee_addr[8])
{
  PRINTF("%s: PAN=%x Short Addr=%x\r\n",__FUNCTION__, pan, addr);

  uint8_t abyte;
  abyte = pan & 0xFF;
  hal_register_write(RG_PAN_ID_0, abyte);
  abyte = (pan >> 8 * 1) & 0xFF;
  hal_register_write(RG_PAN_ID_1, abyte);

  abyte = addr & 0xFF;
  hal_register_write(RG_SHORT_ADDR_0, abyte);
  abyte = (addr >> 8 * 1) & 0xFF;
  hal_register_write(RG_SHORT_ADDR_1, abyte);

  if(ieee_addr != NULL) {
    PRINTF("MAC=%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_7, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_6, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_5, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_4, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_3, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_2, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_1, *ieee_addr++);
    PRINTF(":%x", *ieee_addr);
    hal_register_write(RG_IEEE_ADDR_0, *ieee_addr);
    PRINTF("\r\n");
  }
}
/*---------------------------------------------------------------------------*/
static uint8_t
get_channel(void)
{
  return channel;
}
/*---------------------------------------------------------------------------*/
static void
set_channel(uint8_t c)
{
  /* Wait for any transmission to end. */
  if (is_sleeping())
  {
    return;
  }
  PRINTF("rf212_%s: %u\r\n", __FUNCTION__, c);
  wait_idle();
  hal_subregister_write(SR_CHANNEL, c);
}
/*---------------------------------------------------------------------------*/
/* Do not call from ISR context, that will break the SPI bus if any other device
 * drivers are using it! */
bool
at86rf212_interrupt(rtimer_clock_t time)
{
  /* Separate RF212 has a single radio interrupt and the source must be read from the IRQ_STATUS register */
  uint8_t interrupt_source;

  /* Read Interrupt source. */
  interrupt_source = hal_register_read(RG_IRQ_STATUS);

  /* Handle the incoming interrupt. Prioritized. */
  if((interrupt_source & HAL_RX_START_MASK))
  {
    rf212_sfd_start_time = time;
  }
  if(interrupt_source & HAL_TRX_END_MASK)
  {
    volatile uint8_t state = hal_subregister_read(SR_TRX_STATUS);
    if((state == BUSY_RX_AACK) || (state == RX_ON) || (state == BUSY_RX) || (state == RX_AACK_ON))
    {
      /* Received packet interrupt */
      if (hal_subregister_read(SR_RX_CRC_VALID))
      {
        /* Buffer the frame and call poll the radio process */
        if(rxframe[rxframe_head].length == 0)
        {
          hal_frame_read(&rxframe[rxframe_head]);
          rxframe_head++;
          if(rxframe_head >= RF212_RX_BUFFERS)
          {
            rxframe_head = 0;
          }
          return true;
        }
      }
    }
  }
  if(interrupt_source & HAL_TRX_UR_MASK)
  {
  }
  if(interrupt_source & HAL_PLL_UNLOCK_MASK)
  {
  }
  if(interrupt_source & HAL_PLL_LOCK_MASK)
  {
  }
  if(interrupt_source & HAL_BAT_LOW_MASK)
  {
    /*  Disable BAT_LOW interrupt to prevent endless interrupts. The interrupt */
    /*  will continously be asserted while the supply voltage is less than the */
    /*  user-defined voltage threshold. */
    uint8_t trx_isr_mask = hal_register_read(RG_IRQ_MASK);
    trx_isr_mask &= ~HAL_BAT_LOW_MASK;
    hal_register_write(RG_IRQ_MASK, trx_isr_mask);
  }
  return false;
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
