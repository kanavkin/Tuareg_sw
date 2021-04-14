/**
this module covers the ignition hardware layer control
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg.h"
#include "fueling_hw.h"
#include "highspeed_loggers.h"


/******************************************************************************************************************************
injector hardware control
******************************************************************************************************************************/
void set_injector1_powered()
{
    //ON
    gpio_set_pin(GPIOC, 8, PIN_ON);

    Tuareg.flags.fuel_injector_1 = true;
}

void set_injector1_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 8, PIN_OFF);

    Tuareg.flags.fuel_injector_1= false;
}


void set_injector2_powered()
{
    //ON
    gpio_set_pin(GPIOC, 9, PIN_ON);

    Tuareg.flags.fuel_injector_2= true;
}

void set_injector2_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 9, PIN_OFF);

    Tuareg.flags.fuel_injector_2= false;
}

/******************************************************************************************************************************
fuel pump hardware control
******************************************************************************************************************************/

 void set_fuel_pump_powered()
{
    //ON
    gpio_set_pin(GPIOC, 10, PIN_ON);

    Tuareg.flags.fuel_pump= true;
}

 void set_fuel_pump_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 10, PIN_OFF);

    Tuareg.flags.fuel_pump= false;
}



/******************************************************************************************************************************
ignition actuator control
******************************************************************************************************************************/
void set_injector1(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_injector1_powered();
        highspeedlog_register_injector1_power();
    }
    else
    {
        set_injector1_unpowered();
        highspeedlog_register_injector1_unpower();
    }
}


void set_injector2(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_injector2_powered();
        highspeedlog_register_injector2_power();
    }
    else
    {
        set_injector2_unpowered();
        highspeedlog_register_injector2_unpower();
    }
}


void set_fuel_pump(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_fuel_pump_powered();
    }
    else
    {
        set_fuel_pump_unpowered();
    }
}


/******************************************************************************************************************************
ignition hardware initialization

using
-GPIOC8 for injector 1
-GPIOC9 for injector 2
-GPIOC10 for fuel pump

 ******************************************************************************************************************************/
void init_fueling_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    GPIO_configure(GPIOC, 8, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 9, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 10, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    set_injector1_unpowered();
    set_injector2_unpowered();
    set_fuel_pump_unpowered();
}
