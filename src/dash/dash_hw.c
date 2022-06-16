/**
this module covers the dash elements HAL
*/
#include <Tuareg_platform.h>
#include <Tuareg.h>
#include "dash_hw.h"


/******************************************************************************************************************************
dash actuator control
 ******************************************************************************************************************************/

/*
Open drain mode: A “0” in the Output register activates the N-MOS whereas a “1”
in the Output register leaves the port in Hi-Z (the P-MOS is never activated)
*/
void set_mil_hw(actor_control_t level)
{
    if(level == ACTOR_POWERED)
    {
        //gpio_set_pin(GPIOC, 12, PIN_ON);
        gpio_set_pin(GPIOC, 11, PIN_ON);
        Tuareg.flags.mil= true;
    }
    else
    {
        // OFF
        //gpio_set_pin(GPIOC, 12, PIN_OFF);
        gpio_set_pin(GPIOC, 11, PIN_OFF);
        Tuareg.flags.mil= false;
    }
}



/**
    using
    (in later versions hw change:-GPIOC12 for tachometer
    -AF for TIM11 PWM)
    -GPIOC12 for user dash lamp (mil)
    connected to VNLD5090 low side driver
    with open drain control input
*/
void init_dash_hw()
{
    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    //MIL - open drain output to driver IC
    //GPIO_configure(GPIOC, 12, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOC, 11, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    set_mil_hw(ACTOR_UNPOWERED);

    /**
    TACH
    */
    GPIO_configure(GPIOC, 12, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_HIGH, GPIO_PULL_UP);
    GPIO_SetAF(GPIOC, 12, 3);

    //clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    /*
    clock setup
    pwm resolution: 10000 steps
    target freq: ~500 Hz
    */
    TIM11->PSC= (U16) 15;
    TIM11->ARR= (U16) 9999;

    /*
    compare CH 1 setup
    enable PWM mode 1 with preload
    */
    TIM11->CCMR1= TIM_CCMR1_OC1PE | TIM_OCMode_PWM1;
    TIM11->CCER= TIM_CCER_CC1E;
    TIM11->CCR1= (U16) 10;

    //start timer counter
    TIM11->CR1 |= TIM_CR1_CEN;

    //transfer ARR + CC1 shadow registers
    TIM11->EGR= TIM_EGR_UG;
}

void set_tachometer_compare(U32 Compare)
{
    VitalAssert(Compare <= 10000, TID_DASH_HW, 0);

    TIM11->CCR1= (U16) Compare;
}

