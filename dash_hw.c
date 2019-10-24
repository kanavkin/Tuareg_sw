/**
this module covers the dash elements HAL


*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "dash_hw.h"

/******************************************************************************************************************************
fueling actuator control

performance analysis revealed:

 ******************************************************************************************************************************/
inline void set_tachometer(output_pin_t level)
{
    if(level == ON)
    {
        gpio_set_pin(GPIOC, 11, ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 11, OFF);
    }
}


inline void set_user_lamp(output_pin_t level)
{
    if(level == ON)
    {
        gpio_set_pin(GPIOC, 12, ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 12, OFF);
    }
}



/**
    using
    -GPIOC11 for tachometer
    -GPIOC11 for user dash lamp
*/
inline void init_dash_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIO_configure(GPIOC, 11, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 12, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    set_tachometer(OFF);
    set_user_lamp(OFF);
}
