/*   Copyright (c) 2009, Swedish Institute of Computer Science
 *  All rights reserved.
 *
 *
 *	Colin O'Flynn coflynn@newae.com
 *	Eric Gnoske egnoske@gmail.com
 *	Blake Leverett bleverett@gmail.com
 *	Mike Vidales mavida404@gmail.com
 *	Kevin Brown kbrown3@uccs.edu
 *	Nate Bohlmann nate@elfwerks.com
 *	David Kopf dak664@embarqmail.com
 *
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of the copyright holders nor the names of
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/**
 *   \addtogroup wireless
 *  @{
 */

/**
 *   \defgroup hal RF230 hardware level drivers
 *   @{
 */

/**
 *  \file
 *  This file contains low-level radio driver code.
 *  This version is optimized for use with the "barebones" RF230bb driver,
 *  which communicates directly with the contiki core MAC layer.
 *  It is optimized for speed at the expense of generality.
 */
/* #include "contiki-conf.h" */

#if DEBUGFLOWSIZE
extern uint8_t debugflowsize, debugflow[DEBUGFLOWSIZE];
#define DEBUGFLOW(c) if(debugflowsize < (DEBUGFLOWSIZE - 1)) debugflow[debugflowsize++] = c
#else
#define DEBUGFLOW(c)
#endif

/*============================ INCLUDE =======================================*/
#include <stdlib.h>

#include "hal.h"

#include "at86rf230_registermap.h"

#include "stdio.h"

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

/*============================ VARIABLES =====================================*/
/** \brief This is a file internal variable that contains the 16 MSB of the
 *         system time.
 *
 *         The system time (32-bit) is the current time in microseconds. For the
 *         AVR microcontroller implementation this is solved by using a 16-bit
 *         timer (Timer1) with a clock frequency of 1MHz. The hal_system_time is
 *         incremented when the 16-bit timer overflows, representing the 16 MSB.
 *         The timer value it self (TCNT1) is then the 16 LSB.
 *
 *  \see hal_get_system_time
 */
volatile extern signed char rf230_last_rssi;

/*============================ CALLBACKS =====================================*/

/** \brief This function is called when a rx_start interrupt is signaled.
 *
 *         If this function pointer is set to something else than NULL, it will
 *         be called when a RX_START event is signaled. The function takes two
 *         parameters: timestamp in IEEE 802.15.4 symbols (16 us resolution) and
 *         frame length. The event handler will be called in the interrupt domain,
 *         so the function must be kept short and not be blocking! Otherwise the
 *         system performance will be greatly degraded.
 *
 *  \see hal_set_rx_start_event_handler
 */
/* static hal_rx_start_isr_event_handler_t rx_start_callback; */

/** \brief This function is called when a trx_end interrupt is signaled.
 *
 *         If this function pointer is set to something else than NULL, it will
 *         be called when a TRX_END event is signaled. The function takes one
 *         parameter: timestamp in IEEE 802.15.4 symbols (16 us resolution).
 *         The event handler will be called in the interrupt domain,
 *         so the function must not block!
 *
 *  \see hal_set_trx_end_event_handler
 */
/* static hal_trx_end_isr_event_handler_t trx_end_callback; */

/*============================ IMPLEMENTATION ================================*/

#include "K60.h"
#include "llwu.h"

#define HAL_RF230_ISR() void _isr_portb_pin_detect(void)

/* Convenience macro */
#define RF230_CHIP_SELECT_PIN_MASK (1 << RF230_CHIP_SELECT_PIN)

/** \brief  This function initializes the Hardware Abstraction Layer.
 */
void
hal_init(void)
{
  /*** IO Specific Initialization.****/

  /* Enable PORTC clock gate */
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  PORTE->PCR[6] |= 0x0100;     /* Sleep */

  PTE->PDDR |= 0x0040; /* Setup PTE6 (Sleep) as output */

  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
  PORTB->PCR[9] |= 0x00090100;       /* Set PTB9 (IRQ)    as GPIO with active high interrupt */

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
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
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

  spi_acquire_bus(RF230_SPI_NUM);

  /*** Send Register address and read register content. ***/
  /* Write address */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, &address, NULL, 1, 0);
  /* Read data */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, NULL, &register_value, 0, 1);

  spi_release_bus(RF230_SPI_NUM);

  return register_value;
}
/*----------------------------------------------------------------------------*/
/** \brief  This function writes a new value to one of the radio transceiver's
 *          registers.
 *
 *  \see Look at the at86rf230_registermap.h file for register address definitions.
 *
 *  \param  address Address of register to write.
 *  \param  value   Value to write.
 */
void
hal_register_write(uint8_t address, uint8_t value) /* K60: OK, tested */
{
  /* Add the Register Write command to the address. */
  address = HAL_TRX_CMD_RW | (HAL_TRX_CMD_RADDRM & address);

  spi_acquire_bus(RF230_SPI_NUM);

  /*** Send Register address and write register content. ***/
  /* Write address */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, &address, NULL, 1, 0);
  /* Write data */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, &value, NULL, 1, 0);

  spi_release_bus(RF230_SPI_NUM);
}
/*----------------------------------------------------------------------------*/
/** \brief  This function reads the value of a specific subregister.
 *
 *  \see Look at the at86rf230_registermap.h file for register and subregister
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
 *  \see Look at the at86rf230_registermap.h file for register and subregister
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
  /* Send frame read (long mode) command.*/
  static const uint8_t command = HAL_TRX_CMD_FR;
  uint8_t *rx_data;
  uint8_t frame_length;

  spi_acquire_bus(RF230_SPI_NUM);

  /* Write command */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, &command, NULL, 1, 0);

  /* Read frame length. This includes the checksum. */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, NULL, &frame_length, 0, 1);

  /* Check for correct frame length. Bypassing this test can result in a buffer overrun! */
  if((frame_length < HAL_MIN_FRAME_LENGTH) || (frame_length > HAL_MAX_FRAME_LENGTH)) {
    /* Length test failed */
    rx_frame->length = 0;
    rx_frame->lqi = 0;
    rx_frame->crc = false;
  } else {
    rx_data = (rx_frame->data);
    rx_frame->length = frame_length;

    /* Transfer frame buffer to RAM buffer */
    /* Read frame contents */
    /* the LQI value for this frame is appended to the buffer, hence "frame_length + 1" */
    spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
      SPI_TRANSFER_DONE, NULL, rx_data, 0, frame_length + 1);

    rx_frame->lqi = rx_data[frame_length];

    /* If crc was calculated set crc field in hal_rx_frame_t accordingly.
     * Else show the CRC has passed the hardware check.
     */
    rx_frame->crc = true;
  }

  spi_release_bus(RF230_SPI_NUM);
}
/*----------------------------------------------------------------------------*/
/** \brief  This function will download a frame to the radio transceiver's frame
 *          buffer.
 *
 *  \param  write_buffer    Pointer to data that is to be written to frame buffer.
 *  \param  length          Length of data. The maximum length is 127 bytes.
 */
void
hal_frame_write(uint8_t *write_buffer, uint8_t length) /* TODO: Make sure this is working */
{
  /* Send frame transmitt (long mode) command. */
  static const uint8_t command = HAL_TRX_CMD_FW;
#if 0 /* Print frame to uart */
  {
    int i;
    for(i = 0; i < length; i++) {
      printf("%02x ", write_buffer[i]);
      if((i + 1) % 16 == 0) {
        printf("\r\n");
      }
    }
    printf("\r\n");
  }
#endif
  spi_acquire_bus(RF230_SPI_NUM);

  /* Write command */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, &command, NULL, 1, 0);

  /* Write frame length.  */
  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_CONT, &length, NULL, 1, 0);

  /* Write to the Frame Buffer.
   * When the FCS is autogenerated there is no need to transfer the last two bytes
   * since they will be overwritten.
   */
#if !RF230_CONF_CHECKSUM
  length -= 2;
#endif

  spi_transfer_blocking(RF230_SPI_NUM, RF230_CTAS, RF230_CHIP_SELECT_PIN_MASK,
    SPI_TRANSFER_DONE, write_buffer, NULL, length, 0);

  spi_release_bus(RF230_SPI_NUM);
}
/*----------------------------------------------------------------------------*/
/* This #if compile switch is used to provide a "standard" function body for the */
/* doxygen documentation. */
#if defined(DOXYGEN)
/** \brief ISR for the radio IRQ line, triggered by the input capture.
 *  This is the interrupt service routine for timer1.ICIE1 input capture.
 *  It is triggered of a rising edge on the radio transceivers IRQ line.
 */
void RADIO_VECT(void);
#else /* !DOXYGEN */
/* These link to the RF230BB driver in rf230.c */
void rf230_interrupt(void);

extern hal_rx_frame_t rxframe[RF230_CONF_RX_BUFFERS];
extern uint8_t rxframe_head, rxframe_tail;

/* rf230interruptflag can be printed in the main idle loop for debugging */
#define DEBUG 0
#if DEBUG
volatile char rf230interruptflag;
#define INTERRUPTDEBUG(arg) rf230interruptflag = arg
#else
#define INTERRUPTDEBUG(arg)
#endif

/* Separate RF230 has a single radio interrupt and the source must be read from the IRQ_STATUS register */
HAL_RF230_ISR()
{
  /* Clear Interrupt Status Flag */
  BITBAND_REG(PORTB->PCR[9], PORT_PCR_ISF_SHIFT) = 1;    /* Clear interrupt */
  NVIC_ClearPendingIRQ(PORTB_IRQn);
  rf230_interrupt();
}

#endif /* defined(DOXYGEN) */

/** @} */
/** @} */

/*EOF*/
