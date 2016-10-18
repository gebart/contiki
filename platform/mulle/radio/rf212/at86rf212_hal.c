#include "at86rf212_hal.h"
#include "at86rf212.h"

#include "at86rf212_registermap.h"
#include "stdio.h"

#include <stdlib.h>

/*============================ MACROS ========================================*/

/*
 * Macros defined for the radio transceiver's access modes.
 *
 * These functions are implemented as macros since they are used very often.
 */
#define HAL_DUMMY_READ         (0x00) /**<  Dummy value for the SPI. */

#define HAL_TRX_CMD_RW         (0xC0) /**<  Register Write (short mode). */
#define HAL_TRX_CMD_RR         (0x80) /**<  Register Read (short mode). */
#define HAL_TRX_CMD_FW         (0x60) /**<  Frame Transmit Mode (long mode). */
#define HAL_TRX_CMD_FR         (0x20) /**<  Frame Receive Mode (long mode). */
#define HAL_TRX_CMD_SW         (0x40) /**<  SRAM Write. */
#define HAL_TRX_CMD_SR         (0x00) /**<  SRAM Read. */
#define HAL_TRX_CMD_RADDRM     (0x7F) /**<  Register Address Mask. */

#define HAL_CALCULATED_CRC_OK   (0) /**<  CRC calculated over the frame including the CRC field should be 0. */

#define HAL_SPI_TRANSFER_OPEN() HAL_ENTER_CRITICAL_REGION();
#define HAL_SPI_TRANSFER_CLOSE() HAL_LEAVE_CRITICAL_REGION();

#include "K60.h"
#include "llwu.h"

static void hal_isr(void* arg);

static inline void
hal_spi_send(uint8_t data, int cont)
{
  /* Send data */
  if(cont)
  {
    SPI0->PUSHR = SPI_PUSHR_PCS((1 << HAL_SS_PIN)) | SPI_PUSHR_CONT_MASK | data;
  }
  else
  {
    SPI0->PUSHR = SPI_PUSHR_PCS((1 << HAL_SS_PIN)) | data;
  }
  SPI0->SR |= SPI_SR_TCF_MASK;
  while(!(SPI0->SR & SPI_SR_TCF_MASK)) ;

  /* Dummy read */
  SPI0->SR |= SPI_SR_TCF_MASK;
  data = (0xFF & SPI0->POPR);
}

static inline uint8_t
hal_spi_receive(int cont)
{
  /* Dummy write */
  if(cont)
  {
    SPI0->PUSHR = SPI_PUSHR_PCS((1 << HAL_SS_PIN)) | SPI_PUSHR_CONT_MASK;
  }
  else
  {
    SPI0->PUSHR = SPI_PUSHR_PCS((1 << HAL_SS_PIN));
  }
  SPI0->SR |= SPI_SR_TCF_MASK;
  while(!(SPI0->SR & SPI_SR_TCF_MASK)) ;

  /* Read data */
  SPI0->SR |= SPI_SR_TCF_MASK;
  return 0xFF & SPI0->POPR;
}


/** \brief  This function initializes the Hardware Abstraction Layer.
 */
void
hal_init(void)
{
  /*** IO Specific Initialization.****/

  gpio_init(SLPTR_GPIO, GPIO_OUT);
  gpio_init(RST_GPIO, GPIO_OUT);
  gpio_init(PWR_GPIO, GPIO_OUT);

  /*
   * Give radio interrupt highest priority.
   * If lowered it must at least be higher than the the interrupt the radio
   * code may be running from so that wake up and state transition will work.
   */
  NVIC_SetPriority(PORTB_IRQn, 0);
  gpio_init_int(IRQ_GPIO, GPIO_IN, GPIO_RISING, hal_isr, NULL);

  /* Enable power switch to radio */
  hal_set_pwr_high();

  /* Platform specific SPI is initialized in spi-config.c */

  /*** Enable interrupts from the radio transceiver. ***/
  hal_enable_trx_interrupt();
}
/*----------------------------------------------------------------------------*/
/** \brief  This function reads data from one of the radio transceiver's registers.
 *
 *  \param  address Register address to read from. See datasheet for register
 *                  map.
 *
 *  \see Look at the at86rf212_registermap.h file for register address definitions.
 *
 *  \returns The actual value of the read register.
 */
uint8_t
hal_register_read(uint8_t address)
{
  uint8_t register_value = 0;

  /* Add the register read command to the register address. */
  address &= HAL_TRX_CMD_RADDRM;
  address |= HAL_TRX_CMD_RR;

  HAL_ENTER_CRITICAL_REGION();

  /*** Send Register address and read register content. ***/
  hal_spi_send(address, true);   /* Write address */
  register_value = hal_spi_receive(false);   /* Read register */

  HAL_LEAVE_CRITICAL_REGION();

  return register_value;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf212_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void
hal_register_write(uint8_t address, uint8_t value) /* K60: OK, tested */
{
  /* Add the Register Write command to the address. */
  address = HAL_TRX_CMD_RW | (HAL_TRX_CMD_RADDRM & address);

  HAL_ENTER_CRITICAL_REGION();

  /*** Send Register address and write register content. ***/
  hal_spi_send(address, true);   /* Write address */
  hal_spi_send(value, false);   /* Write data */

  HAL_LEAVE_CRITICAL_REGION();
}
/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf212_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position   Bit position of the subregister
 *  \retval Value of the read subregister.
 */
uint8_t
hal_subregister_read(uint8_t address, uint8_t mask, uint8_t position)
{
  /* Read current register value and mask out subregister. */
  uint8_t register_value = hal_register_read(address);
  register_value &= mask;
  register_value >>= position;   /* Align subregister value. */

  return register_value;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          subregisters.
 *
 *  \see Look at the at86rf212_registermap.h file for register and subregister
 *       definitions.
 *
 *  \param  address  Main register's address.
 *  \param  mask  Bit mask of the subregister.
 *  \param  position  Bit position of the subregister
 *  \param  value  Value to write into the subregister.
 */
void
hal_subregister_write(uint8_t address, uint8_t mask, uint8_t position,
                      uint8_t value)
{
  /* Read current register value and mask area outside the subregister. */
  uint8_t register_value = hal_register_read(address);
  register_value &= ~mask;

  /* Start preparing the new subregister value. shift in place and mask. */
  value <<= position;
  value &= mask;

  value |= register_value;   /* Set the new subregister value. */

  /* Write the modified register value. */
  hal_register_write(address, value);
}
/*----------------------------------------------------------------------------*/
/** \brief  Transfer a frame from the radio transceiver to a RAM buffer
 *
 *          This version is optimized for use with contiki RF230BB driver.
 *          The callback routine and CRC are left out for speed in reading the rx buffer.
 *          Any delays here can lead to overwrites by the next packet!
 *
 *          If the frame length is out of the defined bounds, the length, lqi and crc
 *          are set to zero.
 *
 *  \param  rx_frame    Pointer to the data structure where the frame is stored.
 */
void
hal_frame_read(hal_rx_frame_t *rx_frame) /* TODO: Make sure this is working */
{
  uint8_t *rx_data;
  uint8_t frame_length;

  HAL_ENTER_CRITICAL_REGION();

  /*Send frame read (long mode) command.*/
  hal_spi_send(HAL_TRX_CMD_FR, true);

  /*Read frame length. This includes the checksum. */
  frame_length = hal_spi_receive(true);

  /*Check for correct frame length. Bypassing this test can result in a buffer overrun! */
  if((frame_length < HAL_MIN_FRAME_LENGTH) || (frame_length > HAL_MAX_FRAME_LENGTH))
  {
    /* Length test failed */
    rx_frame->length = 0;
    rx_frame->crc = false;
  }
  else
  {
    rx_data = (rx_frame->data);
    rx_frame->length = frame_length;
    /*Transfer frame buffer to RAM buffer */

    do
    {
      *rx_data++ = hal_spi_receive(true);
    } while(--frame_length > 0);

    /* Dummy read */
    hal_spi_receive(false);

    /* If crc was calculated set crc field in hal_rx_frame_t accordingly.
     * Else show the crc has passed the hardware check.
     */
    rx_frame->crc = true;

    /*
     * ED_LEVEL should be used instead of RSSI in extended operation mode so
     * use it always.
     */
    rx_frame->rssi = hal_register_read(RG_ED_LEVEL);
  }

  HAL_LEAVE_CRITICAL_REGION();
}
/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void
hal_frame_write(const uint8_t *write_buffer, uint8_t length) /* TODO: Make sure this is working */
{
  HAL_ENTER_CRITICAL_REGION();

  /* Send frame transmit (long mode) command. */
  hal_spi_send(HAL_TRX_CMD_FW, true);

  /* Sendframe length.  */
  hal_spi_send(length, true);

  /* Download to the Frame Buffer.
   * When the FCS is autogenerated there is no need to transfer the last two bytes
   * since they will be overwritten.
   */
  // Remove crc
  length -= 2;

  do
  {
    /* Do not assert CS on last byte */
    hal_spi_send(*write_buffer++, !(length == 1));
  } while(--length);

  HAL_LEAVE_CRITICAL_REGION();
}
/*----------------------------------------------------------------------------*/
static void hal_isr(void* arg)
{
#ifndef RF212_IRQ_POLL
  if (at86rf212_interrupt(RTIMER_NOW()))
  {
#endif
    at86rf212_poll();
#ifndef RF212_IRQ_POLL
  }
#endif
}
