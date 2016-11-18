/*
 * Interrupt vector for K60 MCU.
 *
 * Note: It is not necessary to modify this file to define custom interrupt
 * service routines. All symbols are defined weak, it is only necessary to
 * define a function with the same name in another file to override the default
 * interrupt handlers.
 */

#include "K60.h"
#include <stdio.h>

#define SECTION(x) __attribute__ ((section(#x)))
#define ISR_VECTOR_SECTION SECTION(.vector_table)
void reset_handler(void) __attribute__((naked));
void isr_nmi(void);
void isr_hardfault(void);
void isr_memmanage(void);
void isr_busfault(void);
void isr_usagefault(void);

/* Default handler for interrupts, infinite loop */
static void unhandled_interrupt(void) __attribute__((unused));

#define UNHANDLED_ALIAS __attribute__((weak, alias("unhandled_interrupt")));

/* __attribute__((naked)) in order to not add any function prologue to the
 * default hardfault handler written in asm */
/* __attribute__((unused)) in order to avoid (incorrect) compiler warnings about
 * the functions being unused when only referenced from the weak alias. */
static void dHardFault_handler(void) __attribute__((naked, unused));
static void dMemManage_handler(void) __attribute__((unused));
static void dUsageFault_handler(void) __attribute__((unused));
static void dBusFault_handler(void) __attribute__((unused));

/* ARM Cortex defined interrupt vectors */
void reset_handler(void) __attribute__((naked));
void isr_nmi(void);
void isr_hardfault(void) __attribute__((weak, alias("dHardFault_handler")));
void isr_memmanage(void) __attribute__((weak, alias("dMemManage_handler")));
void isr_busfault(void) __attribute__((weak, alias("dBusFault_handler")));
void isr_usagefault(void) __attribute__((weak, alias("dUsageFault_handler")));
void isr_reserved(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
/* void isr_reserved(void) UNHANDLED_ALIAS; */
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_svc(void) UNHANDLED_ALIAS;
void isr_debugmonitor(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_pendsv(void) UNHANDLED_ALIAS;
void isr_systick(void) UNHANDLED_ALIAS;

/* device-specific (freescale) defined interrupt vectors */
void isr_dma0_complete(void) UNHANDLED_ALIAS;
void isr_dma1_complete(void) UNHANDLED_ALIAS;
void isr_dma2_complete(void) UNHANDLED_ALIAS;
void isr_dma3_complete(void) UNHANDLED_ALIAS;
void isr_dma4_complete(void) UNHANDLED_ALIAS;
void isr_dma5_complete(void) UNHANDLED_ALIAS;
void isr_dma6_complete(void) UNHANDLED_ALIAS;
void isr_dma7_complete(void) UNHANDLED_ALIAS;
void isr_dma8_complete(void) UNHANDLED_ALIAS;
void isr_dma9_complete(void) UNHANDLED_ALIAS;
void isr_dma10_complete(void) UNHANDLED_ALIAS;
void isr_dma11_complete(void) UNHANDLED_ALIAS;
void isr_dma12_complete(void) UNHANDLED_ALIAS;
void isr_dma13_complete(void) UNHANDLED_ALIAS;
void isr_dma14_complete(void) UNHANDLED_ALIAS;
void isr_dma15_complete(void) UNHANDLED_ALIAS;
void isr_dma_error(void) UNHANDLED_ALIAS;
void isr_mcm(void) UNHANDLED_ALIAS;
void isr_flash_command_complete(void) UNHANDLED_ALIAS;
void isr_flash_read_collision(void) UNHANDLED_ALIAS;
void isr_low_voltage(void) UNHANDLED_ALIAS;
void isr_llwu(void) UNHANDLED_ALIAS;
void isr_watchdog(void) UNHANDLED_ALIAS;
void isr_random_number_generator(void) UNHANDLED_ALIAS;
void isr_i2c0(void) UNHANDLED_ALIAS;
void isr_i2c1(void) UNHANDLED_ALIAS;
void isr_spi0(void) UNHANDLED_ALIAS;
void isr_spi1(void) UNHANDLED_ALIAS;
void isr_spi2(void) UNHANDLED_ALIAS;
void isr_can0_ored_msg_buffer(void) UNHANDLED_ALIAS;
void isr_can0_bus_off(void) UNHANDLED_ALIAS;
void isr_can0_error(void) UNHANDLED_ALIAS;
void isr_can0_tx_warn(void) UNHANDLED_ALIAS;
void isr_can0_rx_warn(void) UNHANDLED_ALIAS;
void isr_can0_wake_up(void) UNHANDLED_ALIAS;
void isr_i2s0_tx(void) UNHANDLED_ALIAS;
void isr_i2s0_rx(void) UNHANDLED_ALIAS;
void isr_can1_ored_msg_buffer(void) UNHANDLED_ALIAS;
void isr_can1_bus_off(void) UNHANDLED_ALIAS;
void isr_can1_error(void) UNHANDLED_ALIAS;
void isr_can1_tx_warn(void) UNHANDLED_ALIAS;
void isr_can1_rx_warn(void) UNHANDLED_ALIAS;
void isr_can1_wake_up(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_uart0_lon(void) UNHANDLED_ALIAS;
void isr_uart0_status(void) UNHANDLED_ALIAS;
void isr_uart0_error(void) UNHANDLED_ALIAS;
void isr_uart1_status(void) UNHANDLED_ALIAS;
void isr_uart1_error(void) UNHANDLED_ALIAS;
void isr_uart2_status(void) UNHANDLED_ALIAS;
void isr_uart2_error(void) UNHANDLED_ALIAS;
void isr_uart3_status(void) UNHANDLED_ALIAS;
void isr_uart3_error(void) UNHANDLED_ALIAS;
void isr_uart4_status(void) UNHANDLED_ALIAS;
void isr_uart4_error(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_adc0(void) UNHANDLED_ALIAS;
void isr_adc1(void) UNHANDLED_ALIAS;
void isr_cmp0(void) UNHANDLED_ALIAS;
void isr_cmp1(void) UNHANDLED_ALIAS;
void isr_cmp2(void) UNHANDLED_ALIAS;
void isr_ftm0(void) UNHANDLED_ALIAS;
void isr_ftm1(void) UNHANDLED_ALIAS;
void isr_ftm2(void) UNHANDLED_ALIAS;
void isr_cmt(void) UNHANDLED_ALIAS;
void isr_rtc_alarm(void) UNHANDLED_ALIAS;
void isr_rtc_seconds(void) UNHANDLED_ALIAS;
void isr_pit0(void) UNHANDLED_ALIAS;
void isr_pit1(void) UNHANDLED_ALIAS;
void isr_pit2(void) UNHANDLED_ALIAS;
void isr_pit3(void) UNHANDLED_ALIAS;
void isr_pdb(void) UNHANDLED_ALIAS;
void isr_usb_otg(void) UNHANDLED_ALIAS;
void isr_usb_charger_detect(void) UNHANDLED_ALIAS;
void isr_enet_1588_timer(void) UNHANDLED_ALIAS;
void isr_enet_tx(void) UNHANDLED_ALIAS;
void isr_enet_rx(void) UNHANDLED_ALIAS;
void isr_enet_error_misc(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_sdhc(void) UNHANDLED_ALIAS;
void isr_dac0(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_tsi(void) UNHANDLED_ALIAS;
void isr_mcg(void) UNHANDLED_ALIAS;
void isr_lptmr0(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_porta(void) UNHANDLED_ALIAS;
void isr_portb(void) UNHANDLED_ALIAS;
void isr_portc(void) UNHANDLED_ALIAS;
void isr_portd(void) UNHANDLED_ALIAS;
void isr_porte(void) UNHANDLED_ALIAS;
/* void isr_reserved(void) UNHANDLED_ALIAS; */
/* void isr_reserved(void) UNHANDLED_ALIAS; */
void isr_software(void) UNHANDLED_ALIAS;

/* This is the value of the stack pointer at power on. This is not a real function. */
void __stack_top(void);

typedef void (*ISR_func)(void);

const ISR_func isr_vector[256] __attribute__((used)) ISR_VECTOR_SECTION =
{
  /* ARM Cortex defined interrupt vectors */
  __stack_top,
  reset_handler,
  isr_nmi,
  isr_hardfault,
  isr_memmanage,
  isr_busfault,
  isr_usagefault,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_svc,
  isr_debugmonitor,
  isr_reserved,
  isr_pendsv,
  isr_systick,

  /* Device-specific (Freescale defined) interrupt vectors */
  isr_dma0_complete,
  isr_dma1_complete,
  isr_dma2_complete,
  isr_dma3_complete,
  isr_dma4_complete,
  isr_dma5_complete,
  isr_dma6_complete,
  isr_dma7_complete,
  isr_dma8_complete,
  isr_dma9_complete,
  isr_dma10_complete,
  isr_dma11_complete,
  isr_dma12_complete,
  isr_dma13_complete,
  isr_dma14_complete,
  isr_dma15_complete,
  isr_dma_error,
  isr_mcm,
  isr_flash_command_complete,
  isr_flash_read_collision,
  isr_low_voltage,
  isr_llwu,
  isr_watchdog,
  isr_random_number_generator,
  isr_i2c0,
  isr_i2c1,
  isr_spi0,
  isr_spi1,
  isr_spi2,
  isr_can0_ored_msg_buffer,
  isr_can0_bus_off,
  isr_can0_error,
  isr_can0_tx_warn,
  isr_can0_rx_warn,
  isr_can0_wake_up,
  isr_i2s0_tx,
  isr_i2s0_rx,
  isr_can1_ored_msg_buffer,
  isr_can1_bus_off,
  isr_can1_error,
  isr_can1_tx_warn,
  isr_can1_rx_warn,
  isr_can1_wake_up,
  isr_reserved,
  isr_uart0_lon,
  isr_uart0_status,
  isr_uart0_error,
  isr_uart1_status,
  isr_uart1_error,
  isr_uart2_status,
  isr_uart2_error,
  isr_uart3_status,
  isr_uart3_error,
  isr_uart4_status,
  isr_uart4_error,
  isr_reserved,
  isr_reserved,
  isr_adc0,
  isr_adc1,
  isr_cmp0,
  isr_cmp1,
  isr_cmp2,
  isr_ftm0,
  isr_ftm1,
  isr_ftm2,
  isr_cmt,
  isr_rtc_alarm,
  isr_rtc_seconds,
  isr_pit0,
  isr_pit1,
  isr_pit2,
  isr_pit3,
  isr_pdb,
  isr_usb_otg,
  isr_usb_charger_detect,
  isr_enet_1588_timer,
  isr_enet_tx,
  isr_enet_rx,
  isr_enet_error_misc,
  isr_reserved,
  isr_sdhc,
  isr_dac0,
  isr_reserved,
  isr_tsi,
  isr_mcg,
  isr_lptmr0,
  isr_reserved,
  isr_porta,
  isr_portb,
  isr_portc,
  isr_portd,
  isr_porte,
  isr_reserved,
  isr_reserved,
  isr_software, /* vector 110 */
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved,
  isr_reserved /* vector 255 */
};

void
isr_nmi(void)
{
#ifdef NMI_WATCHDOG_PIN
  /* This must be done without any function calls since the stack may be corrupt
   * at this point. */
  /* We need to reinitialize the GPIO pin to a known state to avoid situations
   * where the pin is not toggled and we get stuck in a reboot loop */
  /* enable module clock */
  BITBAND_REG32(SIM->SCGC5, SIM_SCGC5_PORTA_SHIFT + (NMI_WATCHDOG_PORT_NUM)) = 1;
  /* Enable GPIO */
  NMI_WATCHDOG_PORT->PCR[NMI_WATCHDOG_PIN] = PORT_PCR_MUX(1);
  /* Set GPIO to OUT */
  NMI_WATCHDOG_GPIO->PDDR = (1 << NMI_WATCHDOG_PIN);
  /* Toggle state */
  NMI_WATCHDOG_GPIO->PTOR = (1 << NMI_WATCHDOG_PIN);
  /* spin for a while to avoid the system reset causing the WDT signal to be
   * released too soon */
  for (unsigned int i = 0; i < 10000; ++i) {
    asm volatile ("nop\n");
  }
  NMI_WATCHDOG_GPIO->PTOR = (1 << NMI_WATCHDOG_PIN);
  /* We can't be sure that any function calls will work in this situation, so we
   * need to do the WDT poking before we can output any debug information */
  printf("NMI: poked WDT\n");
#endif
  printf("NMI handler!\n");
  printf("rebooting...\n");
  NVIC_SystemReset();
  while(1);
}

static void
unhandled_interrupt(void)
{
  /* The IPSR register can be used to identify which interrupt caused the
   * unhandled interrupt handler to be invoked. See Cortex-M4 Devices Generic User Guide. */
  volatile uint32_t ipsr = __get_IPSR();
  (void)ipsr;
  while(1);
}

/**
 * Default handler of Hard Faults
 *
 * This function is only an assembly language wrapper for the function
 * hard_fault_handler_c, defined in fault-handlers.c
 */
static void
dHardFault_handler(void)
{
  __ASM volatile
  (
    "tst lr, #4\n"
    "ite eq\n"
    "mrseq r0, msp\n"
    "mrsne r0, psp\n"
    "b hard_fault_handler_c\n"
  );
  __ASM volatile ("stuckloop%=:\nb stuckloop%=\n":::);
}

/**
 * Default handler of Usage Fault
 */
static void
dUsageFault_handler(void)
{
  while(1);
}

/**
 * Default handler of MemManage Fault
 */
static void
dMemManage_handler(void)
{
  while(1);
}

/**
 * Default handler of Bus Fault
 */
static void
dBusFault_handler(void)
{
  while(1);
}
