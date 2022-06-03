/**
this module covers the dash elements HAL
*/
#include <Tuareg_platform.h>
#include <Tuareg.h>
#include "dash_hw.h"


/******************************************************************************************************************************
dash actuator control
 ******************************************************************************************************************************/
void set_tachometer_compare(U32 Compare)
{
    VitalAssert(Compare < cU16max, TID_DASH_HW, 0);

    TIM11->CCR1= (U16) Compare;
}

/*
Open drain mode: A “0” in the Output register activates the N-MOS whereas a “1”
in the Output register leaves the port in Hi-Z (the P-MOS is never activated)
*/
void set_mil_hw(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        gpio_set_pin(GPIOC, 11, PIN_ON);
        Tuareg.flags.mil= true;
    }
    else
    {
        // OFF
        gpio_set_pin(GPIOC, 11, PIN_OFF);
        Tuareg.flags.mil= false;
    }
}



/**
    using
    -GPIOC12 for tachometer
    -AF for TIM11 PWM
    -GPIOC11 for user dash lamp (mil)
    connected to VNLD5090 low side driver
    with open drain control input
*/
void init_dash_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //open drain output to driver IC
    GPIO_configure(GPIOC, 11, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 12, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    //connect PIN 12 to Timer 11 CH 1
    GPIO_SetAF(GPIOC, 12, 3);

    /**
    init timer for tachometer PWM signal
    */

    //clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    // clear flags
    TIM11->SR= (U16) 0;

    //set prescaler -> f_PWM := 500 Hz
    TIM11->PSC= (U16) 2000 * (SystemCoreClock / 1000000) - 1;

    //enable PWM mode 1 on CH 1 with preload
    TIM11->CCMR1= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;

    //start timer counter
    TIM11->CR1 |= TIM_CR1_CEN;

    TIM11->CCR1= (U16) 0;


    set_mil_hw(ACTOR_UNPOWERED);
}
