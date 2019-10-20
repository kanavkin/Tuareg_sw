/**
this scheduler provides the ignition and fuel subsystem
with a precise time base

it uses the 4 compare channels on 32 bit timer 5 for this

timer resources:
    -ignition channel 1 -> compare channel 1
    -ignition channel 2 -> compare channel 2
    -fuel channel 1 -> compare channel 3
    -fuel channel 2 -> compare channel 4
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "scheduler.h"
#include "ignition_hw.h"

volatile scheduler_t Scheduler;

#include "debug.h"



#warning TODO (oli#4#): implement dummy functions!

static inline void set_fuel_ch1(output_pin_t level)
{

}

static inline void set_fuel_ch2()
{

}

static inline void set_ign_ch1(output_pin_t level)
{
    set_ignition_ch1(level);
}

static inline void set_ign_ch2(output_pin_t level)
{
    set_ignition_ch2(level);
}



void init_scheduler()
{
    //clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    // clear flags
    TIM5->SR= (U16) 0;

    //set prescaler
    TIM5->PSC= (U16) SCHEDULER_PERIOD_US * (SystemCoreClock / 1000000) - 1;

    //start timer counter
    TIM5->CR1 |= TIM_CR1_CEN;

    TIM5->EGR |= TIM_EGR_UG;

    //enable timer 5 irq (prio 2)
    NVIC_SetPriority(TIM5_IRQn, 2UL );
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(TIM5_IRQn);
}


void scheduler_set_channel(scheduler_channel_t target_ch, output_pin_t action, U32 delay_us)
{

    volatile U64 compare;
    VU32 now;
    volatile U64 remain;

    //get current timer value -> crucial part
    now= TIM5->CNT;

    //safety check - clip delay
    if(delay_us > SCHEDULER_MAX_PERIOD_US)
    {
        delay_us= SCHEDULER_MAX_PERIOD_US;

        //TODO log a warning
    }

    //DEBUG
    dwt_set_begin();

    //compare value at delay end in ticks
    compare= now  + (delay_us / SCHEDULER_PERIOD_US);

    switch(target_ch)
    {
        case IGN_CH1:

            TIM5->DIER &= (U16) ~TIM_DIER_CC1IE;

            //store the desired action at delay end
            Scheduler.ign_ch1_action= action;

            if(compare >= 0xFFFFFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */

                //amount of ticks after update event
                remain= compare - 0xFFFFFFFF;


                /**
                we can not set the new compare value right now, if we will see it in this timer cycle
                */
                if(remain < now)
                {
                    //compare already behind, hits after update
                    //preload function not needed
                    TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;
                }
                else
                {
                    //set new compare after update event, use a default value by now
                    TIM5->CCR1= (U32) 0x00;

                    //preload function will load new compare value after update event
                    TIM5->CCMR1 |= TIM_CCMR1_OC1PE;
                }

                //set compare register
                TIM5->CCR1= (U32) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;
                TIM5->CCR1  = (U32) compare;

            }

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC1IF;
            TIM5->DIER |= (U16) TIM_DIER_CC1IE;

#warning TODO (oli#1#): get scheduler to work, 11.5 +/- 0.5 us when 10us was commanded
/*

11.5 +/- 0.5 us @ 10us
21.5 +/- 0.5 us @ 20us


*/

            break;


        case IGN_CH2:

            TIM5->DIER &= (U16) ~TIM_DIER_CC2IE;

            Scheduler.ign_ch2_action= action;

            if(compare >= 0xFFFFFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */
                remain= compare - 0xFFFFFFFF;

                if(remain < now)
                {
                    //compare already behind, hits after update
                    TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;
                }
                else
                {
                    //set new compare after update event
                    TIM5->CCMR1 |= TIM_CCMR1_OC2PE;
                }

                TIM5->CCR2  = (U32) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;
                TIM5->CCR2  = (U32) compare;

            }

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC2IF;
            TIM5->DIER |= (U16) TIM_DIER_CC2IE;
            break;


        case FUEL_CH1:

            TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;

            Scheduler.fuel_ch1_action= action;

            if(compare >= 0xFFFFFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */
                remain= compare - 0xFFFFFFFF;

                if(remain < now)
                {
                    //compare already behind, hits after update
                    TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;
                }
                else
                {
                    //set new compare after update event
                    TIM5->CCMR2 |= TIM_CCMR2_OC3PE;
                }

                TIM5->CCR3  = (U32) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;
                TIM5->CCR3  = (U32) compare;

            }

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC3IF;
            TIM5->DIER |= (U16) TIM_DIER_CC3IE;
            break;


        case FUEL_CH2:

            TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;

            Scheduler.fuel_ch2_action= action;

            if(compare >= 0xFFFFFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */
                remain= compare - 0xFFFFFFFF;

                if(remain < now)
                {
                    //compare already behind, hits after update
                    TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;
                }
                else
                {
                    //set new compare after update event
                    TIM5->CCMR2 |= TIM_CCMR2_OC4PE;
                }

                TIM5->CCR4  = (U32) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;
                TIM5->CCR4  = (U32) compare;

            }

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC4IF;
            TIM5->DIER |= (U16) TIM_DIER_CC4IE;
            break;

    default:
        break;

    }

}



void scheduler_reset_channel(scheduler_channel_t target_ch)
{
    switch(target_ch)
    {
        case IGN_CH1:

            TIM5->DIER &= (U16) ~TIM_DIER_CC1IE;
            TIM5->SR    = (U16) ~TIM_SR_CC1IF;
            break;

        case IGN_CH2:

            TIM5->DIER &= (U16) ~TIM_DIER_CC2IE;
            TIM5->SR    = (U16) ~TIM_SR_CC2IF;
            break;

        case FUEL_CH1:

            TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;
            TIM5->SR    = (U16) ~TIM_SR_CC3IF;
            break;

        case FUEL_CH2:

            TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;
            TIM5->SR    = (U16) ~TIM_SR_CC4IF;
            break;

        default:
            break;

    }
}





void TIM5_IRQHandler(void)
{
    if( TIM5->SR & TIM_SR_CC1IF)
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC1IF;

        //debug
        dwt_set_end();

        /**
        ignition channel 1
        */
        set_ign_ch1(Scheduler.ign_ch1_action);



        //disable compare irq
        TIM5->DIER &= (U16) ~TIM_DIER_CC1IE;
    }


    if( TIM5->SR & TIM_SR_CC2IF)
    {
        /**
        ignition channel 2
        */
        set_ign_ch2(Scheduler.ign_ch2_action);

        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC2IF;

        //disable compare irq
        TIM5->DIER &= (U16) ~TIM_DIER_CC2IE;
    }

    if( TIM5->SR & TIM_SR_CC3IF)
    {
        /**
        fuel channel 1
        */
        set_fuel_ch1(Scheduler.fuel_ch1_action);

        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC3IF;

        //disable compare irq
        TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;
    }

    if( TIM5->SR & TIM_SR_CC4IF)
    {
        /**
        fuel channel 2
        */
        set_fuel_ch2(Scheduler.ign_ch2_action);

        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC4IF;

        //disable compare irq
        TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;
    }

}
