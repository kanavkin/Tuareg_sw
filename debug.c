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
    set_debug_led(OFF);
    set_debug_pin(OFF);
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




/**
full state is so heavy
that it destroys engine timing
print_full_state() takes about 855 us
*/
/*
void print_full_state(volatile ignition_timing_t * intime)
{
    UART_newline();

    UART_Print_U16(intime->rpm);
    UART_Send("rpm, advance: ");
    UART_Print_U8(intime->ignition_advance);
    UART_Send_P(PSTR(" deg, dwell advance: "));
    UART_Print_U16(intime->dwell_advance);
    UART_Send_P(PSTR(" deg, on: "));

            switch(intime->coil_on_pos)
            {
            case POSITION_A1:
                UART_Send("A1");
                break;
            case POSITION_A2:
                UART_Send("A2");
                break;
                case POSITION_B1:
                UART_Send("B1");
                break;
                case POSITION_B2:
                UART_Send("B2");
                break;
                case POSITION_C1:
                UART_Send("C1");
                break;
                case POSITION_C2:
                UART_Send("C2");
                break;
                case POSITION_D1:
                UART_Send("D1");
                break;
                case POSITION_D2:
                UART_Send("D2");
                break;
                case UNDEFINED_POSITION:
                break;
            }

            UART_Tx('-');
            UART_Print_U8(intime->coil_on_timing);

            UART_Send_P(PSTR("off:"));

            switch(intime->coil_off_pos)
            {
            case POSITION_A1:
                UART_Send("A1");
                break;
            case POSITION_A2:
                UART_Send("A2");
                break;
                case POSITION_B1:
                UART_Send("B1");
                break;
                case POSITION_B2:
                UART_Send("B2");
                break;
                case POSITION_C1:
                UART_Send("C1");
                break;
                case POSITION_C2:
                UART_Send("C2");
                break;
                case POSITION_D1:
                UART_Send("D1");
                break;
                case POSITION_D2:
                UART_Send("D2");
                break;
                case UNDEFINED_POSITION:
                break;
            }

            UART_Tx('-');
            UART_Print_U8(intime->coil_off_timing);
}
*/


void print_minimal_state(USART_TypeDef * Port, volatile ignition_timing_t * intime)
{
    UART_transmit(Port,'\r');
    UART_Print_U(Port, intime->rpm, TYPE_U32, NO_PAD);
    UART_write(Port, "rpm, advance ");
    UART_Print_U(Port, intime->ignition_advance_deg, TYPE_U32, NO_PAD);
    UART_write(Port, "deg");
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


void print_sensor_data()
{
    //can be called cyclic by adding a lowspeed action

    //intro
    UART_Send(DEBUG_PORT, "\r\nsensors:");


    UART_Send(DEBUG_PORT, "\r\nMAP:");

    if(Tuareg.sensor_interface->active_sensors & ASENSOR_MAP_ACT)
    {
        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->MAP, TYPE_U16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
    }

    UART_Send(DEBUG_PORT, "\r\nBARO:");
    if(Tuareg.sensor_interface->active_sensors & ASENSOR_BARO_ACT)
    {
        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->BARO, TYPE_U16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
    }


    UART_Send(DEBUG_PORT, "\r\nO2:");
    if(Tuareg.sensor_interface->active_sensors & ASENSOR_O2_ACT)
    {
        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->O2, TYPE_U16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
    }


    UART_Send(DEBUG_PORT, "\r\nTPS:");
    if(Tuareg.sensor_interface->active_sensors & ASENSOR_TPS_ACT)
    {
        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->TPS, TYPE_U16, NO_PAD);

        UART_Send(DEBUG_PORT, "\r\nddt_TPS:");
        UART_Print_S(DEBUG_PORT, Tuareg.sensor_interface->ddt_TPS, TYPE_S16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
        UART_Send(DEBUG_PORT, "\r\nddt_TPS:");
        UART_Tx(DEBUG_PORT, '-');
    }


    UART_Send(DEBUG_PORT, "\r\nIAT:");
    if(Tuareg.sensor_interface->active_sensors & ASENSOR_IAT_ACT)
    {
        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->IAT, TYPE_U16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
    }


    UART_Send(DEBUG_PORT, "\r\nCLT:");
    if(Tuareg.sensor_interface->active_sensors & ASENSOR_CLT_ACT)
    {
        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->CLT, TYPE_U16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
    }


    UART_Send(DEBUG_PORT, "\r\nVBAT:");
    if(Tuareg.sensor_interface->active_sensors & ASENSOR_VBAT_ACT)
    {

        UART_Print_U(DEBUG_PORT, Tuareg.sensor_interface->VBAT, TYPE_U16, NO_PAD);
    }
    else
    {
        UART_Tx(DEBUG_PORT, '-');
    }

    //digital sensors
    UART_Send(DEBUG_PORT, "\r\nRUN, CRASH, SIDEST: ");

    if(Tuareg.sensor_interface->digital_sensors & DSENSOR_RUN)
    {
        UART_Tx(DEBUG_PORT, '1');
    }
    else
    {
        UART_Tx(DEBUG_PORT, '0');
    }

    UART_Tx(DEBUG_PORT, '-');

    if(Tuareg.sensor_interface->digital_sensors & DSENSOR_CRASH)
    {
        UART_Tx(DEBUG_PORT, '1');
    }
    else
    {
        UART_Tx(DEBUG_PORT, '0');
    }

    UART_Tx(DEBUG_PORT, '-');

#warning TODO (oli#4#): adjust to current sensor list

    if(Tuareg.sensor_interface->digital_sensors & DSENSOR_DEBUG)
    {
        UART_Tx(DEBUG_PORT, '1');
    }
    else
    {
        UART_Tx(DEBUG_PORT, '0');
    }


}








