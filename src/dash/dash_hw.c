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
void set_tachometer_compare(U32 Compare)
{
    VitalAssert(Compare < cU16max, TID_DASH_HW, 0);

    TIM11->CCR1= (U16) Compare;
}
*/


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

    //MIL - open drain output to driver IC
    GPIO_configure(GPIOC, 11, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    set_mil_hw(ACTOR_UNPOWERED);

    /*
    //TACH
    //GPIO_configure(GPIOC, 12, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_MID, GPIO_PULL_UP);
    //GPIO_configure(GPIOC, 12, GPIO_MODE_OUT, GPIO_OUT_OD, GPIO_SPEED_MID, GPIO_PULL_UP);

    //connect PIN 12 to Timer 11 CH 1
    //GPIO_SetAF(GPIOC, 12, 3);

    init timer for tachometer PWM signal

    //clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    //set prescaler -> f_PWM := 500 Hz
    //(100000000 / 65565 ~ 1500)
    //TIM11->PSC= (U16) 2000 * (SystemCoreClock / 1000000) - 1;
    TIM11->PSC= (U16) 9999;

    //ARR
    TIM11->ARR= (U16) 9999;
    //TIM11->CR1 = TIM_CR1_ARPE;

    //enable PWM mode 1 on CH 1 with preload
    //TIM11->CCMR1= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    //TIM11->CCMR1= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1;
    //TIM11->CCMR1= TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
    TIM11->CCMR1= TIM_CCMR1_OC1PE | TIM_OCMode_PWM1;

    //enable output
    TIM11->CCER= TIM_CCER_CC1E;

    //debug irq
    //TIM11->DIER= TIM_DIER_CC1IE | TIM_DIER_UIE;


    //TIM11->CCR1= (U16) 0;
    TIM11->CCR1= (U16) 3000;

    //transfer ARR + CC1 shadow registers
    TIM11->EGR= TIM_EGR_UG;

    //start timer counter
    TIM11->CR1 |= TIM_CR1_CEN;

    //debug nvic

    //enable timer 11 compare irq (prio 15)
    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 15UL );
    NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
    */

}

/*
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{


    //timer channel 1
    if(TIM11->SR & TIM_SR_CC1IF)
    {
        //clear irq pending bit
        TIM11->SR= (U16) ~TIM_SR_CC1IF;

        //debug
        //gpio_set_pin(GPIOC, 12, PIN_TOGGLE);
        //gpio_set_pin(GPIOC, 12, PIN_OFF);

    }


    if(TIM11->SR & TIM_SR_UIF)
    {
        //clear irq pending bit
        TIM11->SR= (U16) ~TIM_SR_UIF;

        //debug
        //gpio_set_pin(GPIOC, 12, PIN_ON);
    }

}
*/



