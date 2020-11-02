/**
this module contains all the debug tool
that should never be integrated to a
production version
*/

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "debug.h"
#include "decoder_logic.h"
#include "ignition_logic.h"
#include "uart.h"
#include "conversion.h"
#include "sensors.h"
#include "Tuareg.h"



volatile char print_buffer[10];

VU32 Debug_DWT_Timestamp_1 =0;
VU32 Debug_DWT_Timestamp_2 =0;
VU32 Debug_DWT_Delay =0;
VU32 Debug_DWT_End =0;


/**
dbug led on GPIOC-1
(this is led2 on nucleo64)
*/
void set_debug_led(volatile output_pin_t level)
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
    Debug_DWT_Timestamp_1= DWT->CYCCNT;
    Debug_DWT_End= 0;
}


inline void dwt_set_end()
{
    Debug_DWT_Timestamp_2= DWT->CYCCNT;
    Debug_DWT_End= 0xFF;
}


void print_dwt_delay()
{
    if(Debug_DWT_Timestamp_2 > Debug_DWT_Timestamp_1)
    {
        Debug_DWT_Delay= Debug_DWT_Timestamp_2 - Debug_DWT_Timestamp_1;
    }
    else
    {
        Debug_DWT_Delay= (0xFFFFFFFF - Debug_DWT_Timestamp_1) + Debug_DWT_Timestamp_2;
    }


    UART_Send(DEBUG_PORT, "\r\ndelay: ");
    UART_Print_U(DEBUG_PORT, Debug_DWT_Delay, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT, " ticks -> ");
    UART_Print_U(DEBUG_PORT, Debug_DWT_Delay / 100, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT, " us");
}


inline void poll_dwt_printout()
{
    if(Debug_DWT_End)
    {
        print_dwt_delay();
        Debug_DWT_End =0;
    }
}




void delay_us(U32 delay)
{
    U32 target;

    target= DWT->CYCCNT + delay * (SystemCoreClock / 1000000);

    while( DWT->CYCCNT < target);
}


void print_sensor_data(USART_TypeDef * Port)
{
    //can be called cyclic by adding a lowspeed action
    U32 sensor;

    UART_Send(Port, "\r\nO2   TPS   IAT   CLT  VBAT KNOCK  BARO  GEAR  MAP\r\n");

    for(sensor=0; sensor < ASENSOR_COUNT; sensor++)
    {
        if(Tuareg.sensors->asensors_health & (1<< sensor))
        {
            UART_Print_F32(Port, Tuareg.sensors->asensors[sensor]);
        }
        else
        {
            UART_Send(Port, " - ");
        }
    }

    UART_Send(Port, "\r\n(raw:)\r\n");

    for(sensor=0; sensor < ASENSOR_COUNT; sensor++)
    {
        if(Tuareg.sensors->asensors_health & (1<< sensor))
        {
            UART_Print_U(Port, Tuareg.sensors->asensors_raw[sensor], TYPE_U16, PAD);
        }
        else
        {
            UART_Send(Port, " - ");
        }
    }


    UART_Send(Port, "\r\n");
    UART_Send(Port, "\r\nDIGITAL: SPARE2 NEUTRAL RUN CRASH DEBUG\r\n");

    for(sensor=0; sensor < DSENSOR_COUNT; sensor++)
    {
        if(Tuareg.sensors->dsensors & (1<< sensor))
        {
            UART_Tx(Port, '1');
        }
        else
        {
            UART_Tx(Port, '0');
        }

        UART_Tx(Port, ' ');
    }

}


void print_decoder_statistics()
{
    /*
    UART_Send(DEBUG_PORT,  "synced: ");
    UART_Print_U(DEBUG_PORT, Tuareg.decoder_internals->diag_positions_crank_synced, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT,  "async: ");
    UART_Print_U(DEBUG_PORT, Tuareg.decoder_internals->diag_positions_crank_async, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT,  "sync lost events: ");
    UART_Print_U(DEBUG_PORT, Tuareg.decoder_internals->diag_sync_lost_events, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT,  "timeout: ");
    UART_Print_U(DEBUG_PORT, Tuareg.decoder_internals->timeout_count, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT,  "rpm: ");
    UART_Print_U(DEBUG_PORT, Tuareg.decoder->engine_rpm, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT,  "\r\n");
    */
}




