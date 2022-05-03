/**
this module covers the dash elements HAL


*/
#include <Tuareg_platform.h>
#include "act_hw.h"

/******************************************************************************************************************************
spare actuator control

performance analysis revealed:

 ******************************************************************************************************************************/
void set_act1(output_pin_t level)
{
    if(level == PIN_ON)
    {
        gpio_set_pin(GPIOB, 8, PIN_OFF);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOB, 8, PIN_ON);
    }
}


void set_act2(output_pin_t level)
{
    if(level == PIN_ON)
    {
        gpio_set_pin(GPIOB, 9, PIN_OFF);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOB, 9, PIN_ON);
    }
}



/**
    using
    -GPIOB8 for act1
    -GPIOB9 for act2
    connected to VNLD5090 low side driver
    with open drain control input
*/
void init_act_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIO_configure(GPIOB, 8, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOB, 9, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    set_act1(PIN_OFF);
    set_act2(PIN_OFF);
}
