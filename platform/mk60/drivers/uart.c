/**
 * \file
 *         Provide common UART routines for MK60DZ10.
 * \author
 *         Tony Persson <tony.persson@rubico.com>
 */

#include "MK60N512VMD100.h"
#include "uart.h"

/*
 * Initialize UART1 to baud 115200
 */
void uart_init(void)
{
  SIM_SCGC5  |= SIM_SCGC5_PORTC_MASK;
  /* PORTE_PCR25: ISF=0,MUX=3 */
  PORTC_PCR3 = ((PORTC_PCR25 & ~0x01000400) | 0x00000300);

  /* PORTE_PCR24: ISF=0,MUX=3 */
  PORTC_PCR4 = ((PORTC_PCR24 & ~0x01000400) | 0x00000300);

  /* SIM_SCGC4 */
  SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;

  UART1_BDH    = 0;
  UART1_BDL    = 11;
  UART1_C4     = 24;
  UART1_C2     = 0x0C;

//  /* UART1_C2: TE=0,RE=0 */
//  UART1_C2 &= ~0x0C;
//
//  /* UART1_BDL: SBR=0x1A */
//  UART1_BDL = 11;//0x1A;
//
//  /* UART1_BDH: LBKDIE=0,RXEDGIE=0,SBR=0 */
//  UART1_BDH = 0x00;
//
//  /* UART1_MA1: MA=0 */
//  UART1_MA1 = 0x00;
//
//  /* UART1_MA2: MA=0 */
//  UART1_MA2 = 0x00;
//
//  /* UART1_C4: MAEN1=0,MAEN2=0,M10=0,BRFA=1 */
//  UART1_C4 = 0x01;
//
//  /* UART1_C1: LOOPS=0,UARTSWAI=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
//  UART1_C1 = 0x00;
//
//  /* UART1_S2: LBKDIF=1,RXEDGIF=1,MSBF=0,RXINV=0,RWUID=0,BRK13=0,LBKDE=0,RAF=0 */
//  UART1_S2 = 0xC0;
//
//  /* UART1_MODEM: ??=0,RXRTSE=0,TXRTSPOL=0,TXRTSE=0,TXCTSE=0 */
//  UART1_MODEM = 0x00;
//
//  /* UART1_IR: ??=0,IREN=0,TNP=0 */
//  UART1_IR = 0x00;
//
//  /* UART1_TWFIFO: TXWATER=0 */
//  UART1_TWFIFO = 0x00;
//
//  /* UART1_RWFIFO: RXWATER=1 */
//  UART1_RWFIFO = 0x01;
//
//  /* UART1_SFIFO: TXEMPT=0,RXEMPT=0,TXOF=1,RXUF=1 */
//  UART1_SFIFO = 0x03;
//
//  /* UART1_CFIFO: TXFLUSH=1,RXFLUSH=1,TXOFE=0,RXUFE=0 */
//  UART1_CFIFO = 0xC0;
//
//  /* UART1_PFIFO: TXFE=0,RXFE=0 */
//  UART1_PFIFO &= ~0x88;
//
//  /* Dummy read of the UART1_S1 register to clear flags */
//  (void) UART1_S1;
//
//  /* Dummy read of the UART1_D register to clear flags */
//  (void) UART1_D;
//
//  /* UART1_C5: TDMAS=0,RDMAS=0 */
//  UART1_C5 = 0x00;
//
//  /* UART1_C3: R8=0,T8=0,TXDIR=0,TXINV=0,ORIE=0,NEIE=0,FEIE=0,PEIE=0 */
//  UART1_C3 = 0x00;
//
//  /* UART1_C2: TIE=0,TCIE=0,RIE=0,ILIE=0,TE=1,RE=0,RWU=0,SBK=0 */
//  UART1_C2 = 0x08;
}

/*
 * Send char on UART1.
 */
void uart_putchar(char ch)
{
  /* Wait until space is available in the FIFO */
  while (!(UART1_S1 & UART_S1_TDRE_MASK));

  /* Send the character */
  UART1_D = ch;
}

/*
 * Send string to UART1.
 */
void uart_putstring(char *str)
{
  char *p = str;
  while(*p)
    uart_putchar(*p++);
}
