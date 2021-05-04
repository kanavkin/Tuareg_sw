/**
this module covers the ignition hardware layer
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "ignition_hw.h"

#include "Tuareg.h"

#include "highspeed_loggers.h"


/******************************************************************************************************************************
ignition actuator control - helper functions
******************************************************************************************************************************/
void set_coil1_powered()
{
    //ON
    gpio_set_pin(GPIOC, 6, PIN_ON);

    Tuareg.flags.ignition_coil_1= true;
}

void set_coil1_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 6, PIN_OFF);

    Tuareg.flags.ignition_coil_1= false;
}


void set_coil2_powered()
{
    //ON
    gpio_set_pin(GPIOC, 7, PIN_ON);

    Tuareg.flags.ignition_coil_2= true;
}

void set_coil2_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 7, PIN_OFF);

    Tuareg.flags.ignition_coil_2= false;
}


/******************************************************************************************************************************
ignition irq control - helper function
******************************************************************************************************************************/
void trigger_ignition_irq()
{
    ///trigger sw irq
    EXTI->SWIER= EXTI_SWIER_SWIER3;
}



/******************************************************************************************************************************
ignition actuator control

a spark will be generated when switching from powered to unpowered state

 ******************************************************************************************************************************/
void set_ignition_ch1(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_coil1_powered();
        highspeedlog_register_coil1_power();
    }
    else
    {
        set_coil1_unpowered();
        highspeedlog_register_coil1_unpower();

        //prepare irq
       // Tuareg.flags.ign1_irq_flag= true;
       // trigger_ignition_irq();
    }
}


void set_ignition_ch2(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_coil2_powered();
        highspeedlog_register_coil2_power();
    }
    else
    {
        set_coil2_unpowered();
        highspeedlog_register_coil2_unpower();

        //prepare irq
       // Tuareg.flags.ign2_irq_flag= true;
       // trigger_ignition_irq();
    }
}



/******************************************************************************************************************************
ignition hardware initialization

using
    -GPIOC6 for ignition coil 1
    -GPIOC7 for ignition coil 2

    -use EXTI IRQ 3 after spark has fired

 ******************************************************************************************************************************/
void init_ignition_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //coil1,2
    GPIO_configure(GPIOC, 6, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 7, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    //sw irq on exti line 3
    EXTI->IMR |= EXTI_IMR_MR3;

    //enable sw exti irq (prio 10)
    NVIC_SetPriority(EXTI3_IRQn, 10UL);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

    //this will not trigger the ignition irq right away
    set_coil1_unpowered();
    set_coil2_unpowered();
}
