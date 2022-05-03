/**
this module contains all the debug tool
that should never be integrated to a
production version
*/

#include <Tuareg_platform.h>
#include <Tuareg.h>

#include "debug.h"


/**
dbug led on GPIOC-1
(this is led2 on nucleo64)
*/
void set_debug_led(output_pin_t level)
{
    gpio_set_pin(GPIOC, 1, level);
}



/**
dbug pin on GPIOA-8
*/
void set_debug_pin(output_pin_t level)
{
    gpio_set_pin(GPIOA, 8, level);
}



void init_debug_pins()
{
    //Enable PORT A clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN;

    //set output mode
    GPIO_configure(GPIOC, 1, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOA, 8, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    //clear
    set_debug_led(PIN_OFF);
    set_debug_pin(PIN_OFF);
}



void enable_sysclk_check()
{
    /*
    MCO_2 is AF0 on GPIOC-9
    set to output SysClock / 5
    revealed:
    SYSCLK= 100 MHz

    show in gdb: p/t RCC->CFGR
    */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    GPIO_SetAF(GPIOC, 9, 0);
    GPIO_configure(GPIOC, 9, GPIO_MODE_AF, GPIO_OUT_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_NONE);
    RCC->CFGR |= RCC_MCO2Div_5 | RCC_CFGR_MCO2EN;
}


/*!
 * \brief Initialize the SWO trace port for debug message printing
 * \param portBits Port bit mask to be configured
 * \param cpuCoreFreqHz CPU core clock frequency in Hz
 */
void SWO_Init(U32 portBits, U32 cpuCoreFreqHz)
{
  U32 SWOSpeed = 64000; /* default 64k baud rate */
  U32 SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */

  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* enable trace in core debug */

  *((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding) */
  *((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
  *((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC */

  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; /* ITM Trace Control Register */
  ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* ITM Trace Privilege Register */
  ITM->TER = portBits; /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */

  *((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; /* DWT_CTRL */
  *((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; /* Formatter and Flush Control Register */
}


/*!
 * \brief Sends a character over the SWO channel
 * \param c Character to be sent
 * \param portNo SWO channel number, value in the range of 0 to 31
 */
void SWO_PrintChar(char c, U8 portNo)
{

  VU32 timeout;

  /* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
  if ((ITM->TCR&ITM_TCR_ITMENA_Msk) == 0) { /* check Trace Control Register if ITM trace is enabled*/
    return; /* not enabled? */
  }

  /* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
  if ((ITM->TER & (1ul<<portNo))==0) { /* check Trace Enable Register if requested port is enabled */
    return; /* requested port not enabled? */
  }

  timeout = 5000; /* arbitrary timeout value */

  while (ITM->PORT[0].u32 == 0) {
    /* Wait until STIMx is ready, then send data */
    timeout--;
    if (timeout==0) {
      return; /* not able to send */
    }
  }

  ITM->PORT[0].u16 = 0x08 | (c<<8);
}


/*!
 * \brief Sends a string over SWO to the host
 * \param s String to send
 * \param portNumber Port number, 0-31, use 0 for normal debug strings
 */
void SWO_PrintString(const char *s, U8 portNumber)
{
  while (*s!='\0') {
    SWO_PrintChar(*s++, portNumber);
  }
}
