/**
this module covers the ignition HAL


*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "ignition_hw.h"
#include "diagnostics.h"

/******************************************************************************************************************************
ignition actuator control

to trigger the ignition irq use COIL_IGNITION!
(prevent triggering the irq when turning system to stand by)

performance analysis revealed:
one set_ignition_ch1(ON) + set_ignition_ch1(OFF) cycle generates a pulse of about 1 us +/- 50ns
execution time about 120 cycles
 ******************************************************************************************************************************/
inline void set_ignition_ch1(coil_ctrl_t level)
{
    if(level == COIL_DWELL)
    {
        //ON
        gpio_set_pin(GPIOC, 6, PIN_ON);
    }
    else if(level == COIL_IGNITION)
    {
        // OFF
        gpio_set_pin(GPIOC, 6, PIN_OFF);

        trigger_ignition_irq();
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 6, PIN_OFF);
    }
}


inline void set_ignition_ch2(coil_ctrl_t level)
{
   if(level == COIL_DWELL)
    {
        //ON
        gpio_set_pin(GPIOC, 7, PIN_ON);
    }
    else if(level == COIL_IGNITION)
    {
        // OFF
        gpio_set_pin(GPIOC, 7, PIN_OFF);

        trigger_ignition_irq();
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 7, PIN_OFF);
    }
}

/******************************************************************************************************************************
ignition irq control

 ******************************************************************************************************************************/
inline void trigger_ignition_irq()
{
    ignhw_diag_log_event(IGNHWDIAG_SWIER3_TRIGGERED);

    /**
    trigger sw irq
    for ignition timing recalculation
    */
    EXTI->SWIER= EXTI_SWIER_SWIER3;
}



/**
    using
    -GPIOC6 for ignition coil 1

    -use EXTI IRQ 2 for ignition timing
     recalculation after spark has fired

*/
inline void init_ignition_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //coil1,2
    GPIO_configure(GPIOC, 6, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 7, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    set_ignition_ch1(COIL_POWERDOWN);
    set_ignition_ch2(COIL_POWERDOWN);

    //sw irq on exti line 3
    EXTI->IMR |= EXTI_IMR_MR3;

    //enable sw exti irq (prio 10)
    NVIC_SetPriority(EXTI3_IRQn, 10UL);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
}
