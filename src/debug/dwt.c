/**
this module contains all the debug tool
that should never be integrated to a
production version
*/

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"

#include <boctok_types.h>

#include "dwt.h"

#include <uart.h>
#include <uart_printf.h>

VU32 DWT_Timestamp_1 =0;
VU32 DWT_Timestamp_2 =0;
VU32 DWT_Delay =0;
VU32 DWT_End =0;


void dwt_init()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    //start cycle counter
    DWT->CYCCNT= (U32) 0;
    DWT->CTRL |= (U32) 0x01;
}


inline void dwt_stop_cyclecounter()
{
    DWT->CTRL &= (U32) ~0x01;
}


inline U32 dwt_get_cyclecounter()
{
    return DWT->CYCCNT;
}


inline void dwt_set_begin()
{
    DWT_Timestamp_1= DWT->CYCCNT;
    DWT_End= 0;
}


inline void dwt_set_end()
{
    DWT_Timestamp_2= DWT->CYCCNT;
    DWT_End= 0xFF;
}


void print_dwt_delay()
{

    if(DWT_Timestamp_2 > DWT_Timestamp_1)
    {
        DWT_Delay= DWT_Timestamp_2 - DWT_Timestamp_1;
    }
    else
    {
        DWT_Delay= (0xFFFFFFFF - DWT_Timestamp_1) + DWT_Timestamp_2;
    }


    print(DEBUG_PORT, "\r\nDWT delay: ");
    printf_U(DEBUG_PORT, DWT_Delay, NO_PAD);
    print(DEBUG_PORT, " ticks -> ");
    printf_U(DEBUG_PORT, DWT_Delay / 100, NO_PAD);
    print(DEBUG_PORT, " us");
}


inline void poll_dwt_printout()
{
    if(DWT_End)
    {
        print_dwt_delay();
        DWT_End =0;
    }
}


void delay_us(U32 delay)
{
    U32 target;

    target= DWT->CYCCNT + delay * (SystemCoreClock / 1000000);

    while( DWT->CYCCNT < target);
}


/**
calculates the interval since Reference
*/
U32 dwt_get_interval_us(U32 Reference)
{
    U32 now, interval;

    now= dwt_get_cyclecounter();

    if(now > Reference)
    {
        interval= now - Reference;
    }
    else
    {
        interval= (0xFFFFFFFF - Reference) + now;
    }

    return interval / 100;
}


