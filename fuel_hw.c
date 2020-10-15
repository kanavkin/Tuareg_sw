/**
this module covers the ignition HAL


*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "fuel_hw.h"

/******************************************************************************************************************************
fueling actuator control

performance analysis revealed:

 ******************************************************************************************************************************/
inline void set_injector_ch1(output_pin_t level)
{
    if(level == PIN_ON)
    {
        gpio_set_pin(GPIOC, 8, PIN_ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 8, PIN_OFF);
    }
}


inline void set_injector_ch2(output_pin_t level)
{
    if(level == PIN_ON)
    {
        gpio_set_pin(GPIOC, 9, PIN_ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 9, PIN_OFF);
    }
}


inline void set_fuelpump(output_pin_t level)
{
    if(level == PIN_ON)
    {
        gpio_set_pin(GPIOC, 10, PIN_ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 10, PIN_OFF);
    }
}



/**
    using
    -GPIOC8 for injector 1
    -GPIOC9 for injector 2
    -GPIOC10 for fuel pump
*/
inline void init_fuel_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //coil1,2
    GPIO_configure(GPIOC, 8, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 9, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 10, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    set_injector_ch1(PIN_OFF);
    set_injector_ch2(PIN_OFF);
    set_fuelpump(PIN_OFF);
}
