#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"

#include "uart.h"
#include "Tuareg.h"

volatile decoder_hw_t Decoder_hw;

/**
how the decoder works:

-   on initialisation we enable crankpickup irq
-   with first crank pickup irq execution we start crank timer to measure time delay between
    crank pickup events
-   because we know sensing (rising or falling edge), we can tell if a gap or a key on trigger wheel
    has been detected
-   syncronisation we get when a certain time ratio (key/gap ratio) - thats the cycle beginning
-   from cycle beginning every rise/fall event takes us further in engine cycle - decoder keeps track of engine
    position and expects synchronization events after position D2
-   after each sensing event the timer gets a reset
-   a noise filter has been implemented: crank pickup signal edge detection irq gets enabled after some delay time after each
    irq execution to suppress signal noise
-   if a timer overflow occures, the engine most certainly has stalled / is not running

*/


/**
use 16 bit TIM9 for crank pickup signal decoding
*/
void decoder_start_timer()
{
    // clear flags
    TIM9->SR= (U16) 0;

    //set prescaler
    TIM9->PSC= (U16) (DECODER_TIMER_PSC -1);

    //enable output compare for exti
    TIM9->CCR1= (U16) CRANK_NOISE_FILTER;

    //enable overflow interrupt
    TIM9->DIER |= TIM_DIER_UIE;

    //enable compare 1 event
    TIM9->DIER |= TIM_DIER_CC1IE;

    //start timer counter
    TIM9->CR1 |= TIM_CR1_CEN;
}


void decoder_stop_timer()
{
    TIM9->CR1 &= ~TIM_CR1_CEN;
}


/**
crank pickup irq helper functions
(hardware dependent)
*/
void decoder_mask_crank_irq()
{
    EXTI->IMR &= ~EXTI_IMR_MR0;
}

void decoder_unmask_crank_irq()
{
    EXTI->IMR |= EXTI_IMR_MR0;
}


/**
cylinder sensor irq helper functions
(hardware dependent)
*/
void decoder_mask_cis_irq()
{
    EXTI->IMR &= ~EXTI_IMR_MR1;
}

void decoder_unmask_cis_irq()
{
    //clear the pending flag
    EXTI->PR= EXTI_Line1;

    EXTI->IMR |= EXTI_IMR_MR1;
}


/**
pickup sensing helper functions
(the hardware dependent part)
*/
void set_crank_pickup_sensing_rise()
{
    EXTI->RTSR |= EXTI_RTSR_TR0;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
    Decoder_hw.crank_pickup_sensing= RISE;
}

void set_crank_pickup_sensing_fall()
{
    EXTI->FTSR |= EXTI_FTSR_TR0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    Decoder_hw.crank_pickup_sensing= FALL;
}

void set_crank_pickup_sensing_disabled()
{
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
    Decoder_hw.crank_pickup_sensing= DISABLED;
}


/**
select the signal edge that will trigger the decoder (pickup sensing)
(now hardware independent)
masks the crank pickup irq!
*/
void decoder_set_crank_pickup_sensing(sensing_t sensing)
{
    //disable irq for setup
    decoder_unmask_crank_irq();

    switch(sensing)
    {
    case RISE:
                set_crank_pickup_sensing_rise();
                break;

    case FALL:
                set_crank_pickup_sensing_fall();
                break;

    case INVERT:
                if(Decoder_hw.crank_pickup_sensing == RISE)
                {
                    set_crank_pickup_sensing_fall();
                }
                else
                {
                    /*
                    inverting from disabled state will enable trigger on rising edge, too
                    */
                    set_crank_pickup_sensing_rise();
                }

                break;

    default:
                set_crank_pickup_sensing_disabled();
                break;
    }

}


void trigger_decoder_irq()
{
    EXTI->SWIER= EXTI_SWIER_SWIER2;
}







/**
    using
    - GPIOB0 for pickup sensing
    -> EXTI0_IRQ
    enables pickup sensor interrupt

    leaves the decoder hw in the following state:

    - crank sensing according to "SENSING_KEY_BEGIN"
    - crank irq masked
    - cis sensing on rising edge
    - cis irq masked
    - timer configured but stopped
    - decoder irq configured but not triggered



*/
void init_decoder_hw()
{
    //interface variables
    //only variable will be set by decoder_set_crank_pickup_sensing()

    //clock tree setup
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_EXTIEN | RCC_APB2ENR_SYSCFGEN| RCC_APB2ENR_TIM9EN;

    //set input mode for crank pickup and cylinder identification sensor
    GPIO_configure(GPIOB, 0, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
    GPIO_configure(GPIOB, 1, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);

    //map GPIOB0 to EXTI line 0 (crank) and GPIOB1 to EXTI line 1 (cam)
    SYSCFG_map_EXTI(0, EXTI_MAP_GPIOB);
    SYSCFG_map_EXTI(1, EXTI_MAP_GPIOB);

    //configure crank pickup sensor EXTI
    decoder_set_crank_pickup_sensing(SENSING_KEY_BEGIN);

    //configure cylinder identification sensor EXTI (only rising edge), but keep c.i.s. irq masked for now
    EXTI->RTSR |= EXTI_RTSR_TR1;
    EXTI->FTSR &= ~EXTI_FTSR_TR1;
    decoder_mask_cis_irq();

    //sw irq on exti line 2
    EXTI->IMR |= EXTI_IMR_MR2;

    //enable crank pickup irq (prio 1)
    NVIC_SetPriority(EXTI0_IRQn, 1UL);
    NVIC_ClearPendingIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI0_IRQn);

    //enable cis irq (prio 3)
    NVIC_SetPriority(EXTI1_IRQn, 3UL);
    NVIC_ClearPendingIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);

    //enable timer 9 compare 1 irq (prio 1)
    NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 1UL );
    NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);

    //enable sw exti irq (prio 4)
    NVIC_SetPriority(EXTI2_IRQn, 4UL);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);

}



/******************************************************************************************************************************
crankshaft position sensor
 ******************************************************************************************************************************/
void EXTI0_IRQHandler(void)
{
    VU32 timer_buffer;

    /**
    hw dependent part
    */

    //save timer value and start over
    timer_buffer= TIM9->CNT;
    TIM9->CNT= (U16) 0;

    //clear the pending flag after saving timer value to minimize measurement delay
    EXTI->PR= EXTI_Line0;

    /**
    hw independent part
    */
    decoder_logic_crank_handler(timer_buffer);

}


/******************************************************************************************************************************
Timer 9 - decoder control:
    -timer 9 compare event 1 --> enable external interrupt for pickup sensor
    -timer 9 update event --> overflow interrupt occurs when no signal from crankshaft pickup has been received for more then 4s
 ******************************************************************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{

    //TIM9 compare event
    if( TIM9->SR & TIM_IT_CC1)
    {
        /**
        hw dependent part
        */

        //clear the pending flag
        TIM9->SR = (U16) ~TIM_IT_CC1;

        /**
        hw independent part
        */
        decoder_logic_timer_compare_handler();
    }


    //TIM9 update event
    if( TIM9->SR & TIM_IT_Update)
    {
        /**
        hw dependent part
        */

        //clear the pending flag
        TIM9->SR = (U16) ~TIM_IT_Update;

        /**
        hw independent part
        */
        decoder_logic_timer_update_handler();
    }

}


/******************************************************************************************************************************
cylinder identification sensor
 ******************************************************************************************************************************/
void EXTI1_IRQHandler(void)
{
    /**
    hw dependent part
    */

    //clear the pending flag
    EXTI->PR= EXTI_Line1;

    /**
    hw independent part
    */
    decoder_logic_cam_handler();

}



