#ifndef AT86RF212_H
#define AT86RF212_H

#include "contiki.h"
#include "rtimer.h"

/*============================ MACROS ========================================*/
#define RF212B_VERSION                          (3)
#define RF212_SUPPORTED_MANUFACTURER_ID         (31)

#ifdef RF212_CONF_RX_BUFFERS
#define RF212_RX_BUFFERS RF212_CONF_RX_BUFFERS
#else
#define RF212_RX_BUFFERS 1
#endif

#define CHECKSUM_LEN 2
#define AUX_LEN (CHECKSUM_LEN)

/** \brief  This macro defines the start value for the RADIO_* status constants.
 *
 *          It was chosen to have this macro so that the user can define where
 *          the status returned from the TAT starts. This can be useful in a
 *          system where numerous drivers are used, and some range of status codes
 *          are occupied.
 *
 *  \see radio_status_t
 */
#define RADIO_STATUS_START_VALUE                  (0x40)

/** \brief  This enumeration defines the possible return values for the TAT API
 *          functions.
 *
 *          These values are defined so that they should not collide with the
 *          return/status codes defined in the IEEE 802.15.4 standard.
 *
 */
typedef enum
{
  RADIO_SUCCESS = RADIO_STATUS_START_VALUE,    /**< The requested service was performed successfully. */
  RADIO_UNSUPPORTED_DEVICE,           /**< The connected device is not an Atmel AT86RF230. */
  RADIO_INVALID_ARGUMENT,             /**< One or more of the supplied function arguments are invalid. */
  RADIO_TIMED_OUT,                    /**< The requested service timed out. */
  RADIO_WRONG_STATE,                  /**< The end-user tried to do an invalid state transition. */
  RADIO_BUSY_STATE,                   /**< The radio transceiver is busy receiving or transmitting. */
  RADIO_STATE_TRANSITION_FAILED,      /**< The requested state transition could not be completed. */
  RADIO_CCA_IDLE,                     /**< Channel is clear, available to transmit a new frame. */
  RADIO_CCA_BUSY,                     /**< Channel busy. */
  RADIO_TRX_BUSY,                     /**< Transceiver is busy receiving or transmitting data. */
  RADIO_BAT_LOW,                      /**< Measured battery voltage is lower than voltage threshold. */
  RADIO_BAT_OK,                       /**< Measured battery voltage is above the voltage threshold. */
  RADIO_CRC_FAILED,                   /**< The CRC failed for the actual frame. */
  RADIO_CHANNEL_ACCESS_FAILURE,       /**< The channel access failed during the auto mode. */
  RADIO_NO_ACK,                       /**< No acknowledge frame was received. */
} radio_status_t;

/* RF212 radio packet */
typedef struct
{
  uint8_t length;       // Length of frame.
  uint8_t data[ 127 ];  // Actual frame data.
  bool crc;             // Flag - did CRC pass for received frame?
} hal_rx_frame_t;


void at86rf212_interrupt(rtimer_clock_t time);
void at86rf212_set_pan_addr(unsigned pan,
                   unsigned addr,
                   const uint8_t ieee_addr[8]);

extern const struct radio_driver rf212_driver;

#endif // AT86RF212_H
