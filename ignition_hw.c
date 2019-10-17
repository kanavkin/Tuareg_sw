/**
this module covers the ignition HAL


*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "ignition_hw.h"


void set_ignition_ch1(output_pin_t level)
{
    if(level == ON)
    {
        gpio_set_pin(GPIOC, 6, ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 6, OFF);

        /**
        trigger sw irq
        for ignition timing recalculation
        */
        EXTI->SWIER= EXTI_SWIER_SWIER3;
    }
}


void set_ignition_ch2(output_pin_t level)
{
    if(level == ON)
    {
        gpio_set_pin(GPIOC, 7, ON);
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 7, OFF);
    }
}



/**
    using
    -GPIOC6 for ignition coil 1

    -use EXTI IRQ 2 for ignition timing
     recalculation after spark has fired

*/
void init_ignition_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //coil1,2
    GPIO_configure(GPIOC, 6, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 7, GPIO_MODE_OUT, GPIO_OUT_PP, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    set_ignition_ch1(OFF);
    set_ignition_ch2(OFF);

    //sw irq on exti line 3
    EXTI->IMR |= EXTI_IMR_MR3;

    //enable sw exti irq (prio 10)
    NVIC_SetPriority(EXTI3_IRQn, 10UL);
    NVIC_ClearPendingIRQ(EXTI3_IRQn);
    NVIC_EnableIRQ(EXTI3_IRQn);
}
