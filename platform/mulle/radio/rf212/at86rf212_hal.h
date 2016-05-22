#ifndef AT86RF212_HAL_H
#define AT86RF212_HAL_H

/*============================ INCLUDE =======================================*/

#include <stdint.h>
#include <stdbool.h>
#include "K60.h"
/* #include <util/crc16.h> */
#include "contiki-conf.h"
#include "irq.h"
#include "periph/gpio.h"

/*============================ MACROS ========================================*/

/**
 * \name Pin macros
 * \brief These macros convert the platform-specific pin defines into names and functions
 *       that the source code can directly use.
 * \{
 */
#define SLPTR_GPIO GPIO_PIN(PORT_E,  6)
#define RST_GPIO   GPIO_PIN(PORT_C, 12)
#define PWR_GPIO   GPIO_PIN(PORT_D,  7)
#define IRQ_GPIO   GPIO_PIN(PORT_B,  9)

#define hal_set_slptr_high() gpio_set(SLPTR_GPIO)   /**< This macro pulls the SLP_TR pin high. */
#define hal_set_slptr_low()  gpio_clear(SLPTR_GPIO) /**< This macro pulls the SLP_TR pin low. */
#define hal_get_slptr()      gpio_read(SLPTR_GPIO)  /**< Read current state of the SLP_TR pin (High/Low). */
/* rst and pwr is the same */
/* fixed in mulle v0.81, RST pin is connected to PTC12 */
#define hal_set_rst_high()   do { \
  gpio_set(PWR_GPIO); delay_us(0xFFFF); /* Turn on power */ \
  gpio_set(RST_GPIO); \
  } while(0)  /**< This macro pulls the RST pin high. */
#define hal_set_rst_low()   do { \
  gpio_clear(PWR_GPIO); /* Turn off power */ \
  gpio_clear(RST_GPIO); \
  } while(0)  /**< This macro pulls the RST pin low. */
#define hal_get_rst()        gpio_read(RST_GPIO)    /**< Read current state of the RST pin (High/Low). */

#define hal_set_pwr_high()   gpio_set(PWR_GPIO)     /**< This macro pulls the power pin high. */
#define hal_set_pwr_low()    gpio_clear(PWR_GPIO)   /**< This macro pulls the power pin low. */
#define hal_get_pwr()        gpio_read(PWR_GPIO)    /**< Read current state of the RST pin (High/Low). */
#define hal_get_irq()        gpio_read(IRQ_GPIO)    /**< Read current state of the INT pin (High/Low). */
#define HAL_SS_PIN           1                      /**< The slave select pin. */

/** \} */

#define HAL_SS_HIGH()  /* Done in HW on K60 */
#define HAL_SS_LOW()   /* Done in HW on K60 */

#define HAL_ENABLE_RADIO_INTERRUPT() gpio_irq_enable(IRQ_GPIO)
#define HAL_DISABLE_RADIO_INTERRUPT() gpio_irq_disable(IRQ_GPIO)

#define HAL_ENABLE_OVERFLOW_INTERRUPT() ()
#define HAL_DISABLE_OVERFLOW_INTERRUPT() ()

#define HAL_ENTER_CRITICAL_REGION() unsigned int mask = irq_disable()
#define HAL_LEAVE_CRITICAL_REGION() irq_restore(mask)

/** \brief  Enable the interrupt from the radio transceiver.
 */
#define hal_enable_trx_interrupt() HAL_ENABLE_RADIO_INTERRUPT()

/** \brief  Disable the interrupt from the radio transceiver.
 *
 *  \retval 0 if the pin is low, 1 if the pin is high.
 */
#define hal_disable_trx_interrupt() HAL_DISABLE_RADIO_INTERRUPT()

/*============================ TYPDEFS =======================================*/

/*============================ PROTOTYPES ====================================*/

/*============================ MACROS ========================================*/

/** \name Macros for radio operation.
 * \{
 */

#define HAL_MIN_FRAME_LENGTH   (0x03)   /**< A frame should be at least 3 bytes. */
#define HAL_MAX_FRAME_LENGTH   (0x7F)   /**< A frame should no more than 127 bytes. */
/** \} */

/*============================ TYPDEFS =======================================*/
/** \struct hal_rx_frame_t
 *  \brief  This struct defines the rx data container.
 *
 *  \see hal_frame_read
 */
typedef struct {
  uint8_t length;                         /**< Length of frame. */
  uint8_t data[HAL_MAX_FRAME_LENGTH];     /**< Actual frame data. */
  uint8_t lqi;                            /**< LQI value for received frame. */
  uint8_t rssi;
  bool crc;                               /**< Flag - did CRC pass for received frame? */
} hal_rx_frame_t;

/** RX_START event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_rx_start_event_handler(). */
typedef void (*hal_rx_start_isr_event_handler_t)(uint32_t const isr_timestamp, uint8_t const frame_length);

/** RRX_END event handler callback type. Is called with timestamp in IEEE 802.15.4 symbols and frame length. See hal_set_trx_end_event_handler(). */
typedef void (*hal_trx_end_isr_event_handler_t)(uint32_t const isr_timestamp);

typedef void (*rx_callback_t) (uint16_t data);

/*============================ PROTOTYPES ====================================*/
void hal_init(void);

void hal_reset_flags(void);
uint8_t hal_get_bat_low_flag(void);
void hal_clear_bat_low_flag(void);

hal_trx_end_isr_event_handler_t hal_get_trx_end_event_handler(void);
void hal_set_trx_end_event_handler(hal_trx_end_isr_event_handler_t trx_end_callback_handle);
void hal_clear_trx_end_event_handler(void);

hal_rx_start_isr_event_handler_t hal_get_rx_start_event_handler(void);
void hal_set_rx_start_event_handler(hal_rx_start_isr_event_handler_t rx_start_callback_handle);
void hal_clear_rx_start_event_handler(void);

uint8_t hal_get_pll_lock_flag(void);
void hal_clear_pll_lock_flag(void);

uint8_t hal_register_read(uint8_t address);
void hal_register_write(uint8_t address, uint8_t value);
uint8_t hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position);
void hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position, uint8_t value);

/* For speed RF230BB does not use a callback */
void hal_frame_read(hal_rx_frame_t *rx_frame);
void hal_frame_write(uint8_t *write_buffer, uint8_t length);
void hal_sram_read(uint8_t address, uint8_t length, uint8_t *data);
void hal_sram_write(uint8_t address, uint8_t length, uint8_t *data);

#endif

