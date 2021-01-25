/**
this module covers the ignition hardware layer
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "ignition_hw.h"

#include "Tuareg.h"

//#include "diagnostics.h"
//#include "debug.h"

/******************************************************************************************************************************
ignition actuator control

a spark will be generated when switching from powered to unpowered state

performance analysis revealed:
one set_ignition_ch1(ON) + set_ignition_ch1(OFF) cycle generates a pulse of about 1 us +/- 50ns
execution time about 120 cycles
 ******************************************************************************************************************************/
void set_ignition_ch1(actor_control_t level)
{
    if((level == ACTOR_POWERED) && (Tuareg.actors.ignition_inhibit == false))
    {
        //ON
        gpio_set_pin(GPIOC, 6, PIN_ON);

        Tuareg.actors.ignition_coil_1= true;
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 6, PIN_OFF);

        Tuareg.actors.ignition_coil_1= false;

        ///trigger sw irq
        EXTI->SWIER= EXTI_SWIER_SWIER3;
    }
}


void set_ignition_ch2(actor_control_t level)
{
    if((level == ACTOR_POWERED) && (Tuareg.actors.ignition_inhibit == false))
    {
        //ON
        gpio_set_pin(GPIOC, 7, PIN_ON);

        Tuareg.actors.ignition_coil_2= true;
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 7, PIN_OFF);

        Tuareg.actors.ignition_coil_2= false;

        ///trigger sw irq
        EXTI->SWIER= EXTI_SWIER_SWIER3;
    }
}



/******************************************************************************************************************************
ignition irq control

deprecated

 ******************************************************************************************************************************

inline void trigger_ignition_irq()
{
    ignhw_diag_log_event(IGNHWDIAG_SWIER3_TRIGGERED);

    ///trigger sw irq for ignition timing recalculation
    EXTI->SWIER= EXTI_SWIER_SWIER3;
}
*/



/**
    using
    -GPIOC6 for ignition coil 1
    -GPIOC7 for ignition coil 2

    -use EXTI IRQ 3 after spark has fired

*/
void init_ignition_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //coil1,2
    GPIO_configure(GPIOC, 6, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 7, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    set_ignition_ch1(ACTOR_UNPOWERED);
    set_ignition_ch2(ACTOR_UNPOWERED);

    //sw irq on exti line 3
    EXTI->IMR |= EXTI_IMR_MR3;

    //enable sw exti irq (prio 10)
    NVIC_SetPriority(EXTI3_IRQn, 10UL);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);

}
