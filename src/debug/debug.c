/**
this module contains all the debug tool
that should never be integrated to a
production version
*/

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "debug.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


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






