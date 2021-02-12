/**
this module covers the ignition hardware layer
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "ignition_hw.h"

#include "Tuareg.h"


/******************************************************************************************************************************
ignition actuator control - helper functions

- the coils will not be powered, if ignition operation is not permitted
 ******************************************************************************************************************************/
inline void set_ignition_ch1_powered()
{
    if(Tuareg.actors.ignition_inhibit == false)
    {
        //ON
        gpio_set_pin(GPIOC, 6, PIN_ON);

        Tuareg.actors.ignition_coil_1= true;
    }
}

inline void set_ignition_ch1_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 6, PIN_OFF);

    Tuareg.actors.ignition_coil_1= false;
}

inline void set_ignition_ch2_powered()
{
    if(Tuareg.actors.ignition_inhibit == false)
    {
        //ON
        gpio_set_pin(GPIOC, 7, PIN_ON);

        Tuareg.actors.ignition_coil_2= true;
    }
}

inline void set_ignition_ch2_unpowered()
{
    // OFF
    gpio_set_pin(GPIOC, 7, PIN_OFF);

    Tuareg.actors.ignition_coil_2= false;
}


/******************************************************************************************************************************
ignition irq control - helper function

- no irq will be triggered, if ignition operation is not permitted

******************************************************************************************************************************/
inline void trigger_ignition_irq()
{
    if(Tuareg.actors.ignition_inhibit == false)
    {
        ///trigger sw irq
        EXTI->SWIER= EXTI_SWIER_SWIER3;
    }
}


/******************************************************************************************************************************
ignition actuator control

a spark will be generated when switching from powered to unpowered state

 ******************************************************************************************************************************/
void set_ignition_ch1(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_ignition_ch1_powered();
    }
    else
    {
        set_ignition_ch1_unpowered();

        //prepare irq
        Tuareg.actors.ign1_irq_flag= true;
        trigger_ignition_irq();
    }
}


void set_ignition_ch2(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        set_ignition_ch2_powered();
    }
    else
    {
        set_ignition_ch2_unpowered();

        //prepare irq
        Tuareg.actors.ign2_irq_flag= true;
        trigger_ignition_irq();
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

    //this will not trigger the ignition irq right away
    set_ignition_ch1_unpowered();
    set_ignition_ch2_unpowered();

    //sw irq on exti line 3
    EXTI->IMR |= EXTI_IMR_MR3;

    //enable sw exti irq (prio 10)
    NVIC_SetPriority(EXTI3_IRQn, 10UL);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

}
