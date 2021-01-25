/**
this scheduler provides the ignition and fuel subsystem
with a precise time base

it uses the 4 compare channels on 32 bit timer 5 for this

timer resources:
    -ignition channel 1 -> compare channel 1
    -ignition channel 2 -> compare channel 2
    -fuel channel 1 -> compare channel 3
    -fuel channel 2 -> compare channel 4

a timer update event is expected every ~74 min op scheduler operation
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "scheduler.h"
#include "ignition_hw.h"
#include "fuel_hw.h"

#include "diagnostics.h"
#include "Tuareg_errors.h"

#include "uart.h"
#include "uart_printf.h"

volatile scheduler_t Scheduler;


static inline void set_fuel_ch1(output_pin_t level)
{
    set_injector_ch1(level);
}

static inline void set_fuel_ch2(output_pin_t level)
{
    set_injector_ch2(level);
}



void init_scheduler()
{
    //clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    // clear flags
    TIM5->SR= (U16) 0;

    //set prescaler
    TIM5->PSC= (U16) (SCHEDULER_PERIOD_US * (SystemCoreClock / 1000000) - 1);

    //start timer counter
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM5->EGR |= TIM_EGR_UG;

    //enable timer 5 irq (prio 2)
    NVIC_SetPriority(TIM5_IRQn, 2UL );
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(TIM5_IRQn);
}



/**
the scheduler stores the desired action to take
*/
void scheduler_set_channel(scheduler_channel_t Channel, actor_control_t TargetState, U32 Delay_us)
{

    VU64 compare, remain;
    VU32 now;

    /******************************************************
    parameter checks
    ******************************************************/

    //safety check - requested channel
    if(Channel >= SCHEDULER_CH_COUNT)
    {
        return;
    }

    //safety check - clip delay
    if(Delay_us > SCHEDULER_MAX_PERIOD_US)
    {
        Delay_us= SCHEDULER_MAX_PERIOD_US;

        scheduler_diag_log_event(SCHEDIAG_DELAY_CLIPPED);
    }

    if(Delay_us < SCHEDULER_MIN_PERIOD_US)
    {
        /**
        take immediate action - no scheduler allocation actually
        */

///        scheduler_diag_log_event(SCHEDIAG_DELAY_BYPASS);

        switch(Channel)
        {
            case SCHEDULER_CH_IGN1:
                set_ignition_ch1(TargetState);
                break;

            case SCHEDULER_CH_IGN2:
                set_ignition_ch2(TargetState);
                break;
/*
            case SCHEDULER_CH_FUEL1:
                set_fuel_ch1(TargetState);
                break;

            case SCHEDULER_CH_FUEL2:
                set_fuel_ch2(TargetState);
                break;
*/
        }

        //no scheduler allocation
        return;
    }




    /******************************************************
    GO!
    ******************************************************/

    //clean the scheduler channel
    scheduler_reset_channel(Channel);

    //store the desired action at delay end
    Scheduler.targets[Channel]= TargetState;


    __disable_irq();

    //get current timer value -> crucial part
    now= TIM5->CNT;

    //compare value at delay end in ticks
    compare= now  + (Delay_us / SCHEDULER_PERIOD_US);


    switch(Channel)
    {
        case SCHEDULER_CH_IGN1:

            //check if the desired timeout will occur after the next timer update event
            if(compare > 0xFFFFFFFF)
            {
                //amount of ticks after update event
                remain= compare - 0xFFFFFFFF;

                //check if the resulting compare value can be set right now
                if(remain < now)
                {
    ///                scheduler_diag_log_event(SCHEDIAG_ICH1_NEXTC_UPDATE_SET);
                }
                else
                {
                    //preload function will load new compare value after update event
                    TIM5->CCMR1 |= TIM_CCMR1_OC1PE;

     ///               scheduler_diag_log_event(SCHEDIAG_ICH1_NEXTC_PRELOAD_SET);
                }

                //set up the compare register with the remaining value
                TIM5->CCR1= (U32) remain;

                //this should happen only every ~9,5 h

            }
            else
            {
                //set up the compare register with the compare value
                TIM5->CCR1  = (U32) compare;

   ///             scheduler_diag_log_event(SCHEDIAG_ICH1_CURRC_SET);
            }

            if(Tuareg.actors.ignition_scheduler_1 == true)
            {
  ///              scheduler_diag_log_event(SCHEDIAG_ICH1_RETRIGD);
            }

            //save new state
     ///       Scheduler.watchdogs[Channel]= SCHEDULER_WATCHDOG_RESET_VALUE;
    ///        Scheduler.ign1_triggered= true;

            Tuareg.actors.ignition_scheduler_1= true;

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC1IF;
            TIM5->DIER |= (U16) TIM_DIER_CC1IE;

            //collect diagnostic information
     ///       scheduler_diag_log_event(SCHEDIAG_ICH1_SET);

            break;


        case SCHEDULER_CH_IGN2:

            //check if the desired timeout will occur after the next timer update event
            if(compare > 0xFFFFFFFF)
            {
                //amount of ticks after update event
                remain= compare - 0xFFFFFFFF;

                //check if the resulting compare value can be set right now
                if(remain < now)
                {
    ///                scheduler_diag_log_event(SCHEDIAG_ICH2_NEXTC_UPDATE_SET);
                }
                else
                {
                    //preload function will load new compare value after update event
                    TIM5->CCMR1 |= TIM_CCMR1_OC2PE;

     ///               scheduler_diag_log_event(SCHEDIAG_ICH2_NEXTC_PRELOAD_SET);
                }

                //set up the compare register with the remaining value
                TIM5->CCR2= (U32) remain;

                //this should happen only every ~9,5 h

            }
            else
            {
                //set up the compare register with the compare value
                TIM5->CCR2  = (U32) compare;

   ///             scheduler_diag_log_event(SCHEDIAG_ICH2_CURRC_SET);
            }

            if(Tuareg.actors.ignition_scheduler_1 == true)
            {
  ///              scheduler_diag_log_event(SCHEDIAG_ICH2_RETRIGD);
            }

            //save new state
     ///       Scheduler.watchdogs[Channel]= SCHEDULER_WATCHDOG_RESET_VALUE;
    ///        Scheduler.ign2_triggered= true;

            Tuareg.actors.ignition_scheduler_2= true;

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC2IF;
            TIM5->DIER |= (U16) TIM_DIER_CC2IE;

            //collect diagnostic information
     ///       scheduler_diag_log_event(SCHEDIAG_ICH2_SET);


            break;

/*
        case SCHEDULER_CH_FUEL1:

            TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;

            Scheduler.targets[Channel]= TargetState;

            scheduler_diag_log_event(SCHEDIAG_FCH1_SET);

            if(compare >= 0xFFFFFFFF)
            {

                the desired timeout will occur in the next timer cycle

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

                the desired timeout will occur in the current timer cycle

                TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;
                TIM5->CCR3  = (U32) compare;

            }

            if(Scheduler.fuel1_triggered == true)
            {
                scheduler_diag_log_event(SCHEDIAG_FCH1_RETRIGD);
            }

            //save new state
            Scheduler.watchdogs[Channel]= SCHEDULER_WATCHDOG_RESET_VALUE;
            Scheduler.fuel1_triggered= true;


            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC3IF;
            TIM5->DIER |= (U16) TIM_DIER_CC3IE;
            break;


        case SCHEDULER_CH_FUEL2:

            TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;

            Scheduler.targets[Channel]= TargetState;

            scheduler_diag_log_event(SCHEDIAG_FCH2_SET);

            if(compare >= 0xFFFFFFFF)
            {

                the desired timeout will occur in the next timer cycle

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

                the desired timeout will occur in the current timer cycle

                TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;
                TIM5->CCR4  = (U32) compare;

            }

            if(Scheduler.fuel2_triggered == true)
            {
                scheduler_diag_log_event(SCHEDIAG_FCH2_RETRIGD);
            }

            //save new state
            Scheduler.watchdogs[Channel]= SCHEDULER_WATCHDOG_RESET_VALUE;
            Scheduler.fuel2_triggered= true;

            //clear pending flags and enable irq
            TIM5->SR    = (U16) ~TIM_SR_CC4IF;
            TIM5->DIER |= (U16) TIM_DIER_CC4IE;
            break;
*/

    default:
        break;

    }

    __enable_irq();

}



void scheduler_reset_channel(scheduler_channel_t Channel)
{
    switch(Channel)
    {
        case SCHEDULER_CH_IGN1:

            //disable compare irq
            TIM5->DIER &= (U16) ~TIM_DIER_CC1IE;

            //disable compare preload feature
            TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;

            //set temporary compare value
            TIM5->CCR1= (U32) 0xFFFFFFFF;

            //clear irq pending bit
            TIM5->SR= (U16) ~TIM_SR_CC1IF;

            Tuareg.actors.ignition_scheduler_1= false;


            //save new state
   ///         Scheduler.ign1_triggered= false;
   ///         Scheduler.watchdogs[Channel]= 0;


            break;

        case SCHEDULER_CH_IGN2:

            //disable compare irq
            TIM5->DIER &= (U16) ~TIM_DIER_CC2IE;

            //disable compare preload feature
            TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;

            //set temporary compare value
            TIM5->CCR2= (U32) 0xFFFFFFFF;

            //clear irq pending bit
            TIM5->SR= (U16) ~TIM_SR_CC2IF;

            Tuareg.actors.ignition_scheduler_2= false;


            //save new state
   ///         Scheduler.ign2_triggered= false;
   ///         Scheduler.watchdogs[Channel]= 0;
            break;
/*
        case SCHEDULER_CH_FUEL1:

            TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;
            TIM5->SR    = (U16) ~TIM_SR_CC3IF;

            Scheduler.fuel1_triggered= false;
            Scheduler.watchdogs[Channel]= 0;

            scheduler_diag_log_event(SCHEDIAG_FCH1_RESET);
            break;

        case SCHEDULER_CH_FUEL2:

            TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;
            TIM5->SR    = (U16) ~TIM_SR_CC4IF;

            Scheduler.fuel2_triggered= false;
            Scheduler.watchdogs[Channel]= 0;

            scheduler_diag_log_event(SCHEDIAG_FCH2_RESET);
            break;
*/
        default:
            break;

    }
}


void scheduler_update_watchdogs()
{
/*
    if(Scheduler.ign1_triggered == true)
    {
        if(Scheduler.watchdogs[SCHEDULER_CH_IGN1] > 0)
        {
            Scheduler.watchdogs[SCHEDULER_CH_IGN1]--;
        }

        if(Scheduler.watchdogs[SCHEDULER_CH_IGN1] == 0)
        {

            scheduler delay has expired
            MALFUNCTION in scheduler module

            Tuareg_register_scheduler_error();
        }
    }

    if(Scheduler.ign2_triggered == true)
    {
        if(Scheduler.watchdogs[SCHEDULER_CH_IGN2] > 0)
        {
            Scheduler.watchdogs[SCHEDULER_CH_IGN2]--;
        }

        if(Scheduler.watchdogs[SCHEDULER_CH_IGN2] == 0)
        {

            scheduler delay has expired
            MALFUNCTION in scheduler module

            Tuareg_register_scheduler_error();
        }
    }

    if(Scheduler.fuel1_triggered == true)
    {
        if(Scheduler.watchdogs[SCHEDULER_CH_FUEL1] > 0)
        {
            Scheduler.watchdogs[SCHEDULER_CH_FUEL1]--;
        }

        if(Scheduler.watchdogs[SCHEDULER_CH_FUEL1] == 0)
        {

            scheduler delay has expired
            MALFUNCTION in scheduler module

            Tuareg_register_scheduler_error();
        }
    }

    if(Scheduler.fuel2_triggered == true)
    {
        if(Scheduler.watchdogs[SCHEDULER_CH_FUEL2] > 0)
        {
            Scheduler.watchdogs[SCHEDULER_CH_FUEL2]--;
        }

        if(Scheduler.watchdogs[SCHEDULER_CH_FUEL2] == 0)
        {

            scheduler delay has expired
            MALFUNCTION in scheduler module

            Tuareg_register_scheduler_error();
        }
    }
*/
}




void TIM5_IRQHandler(void)
{
    if( TIM5->SR & TIM_SR_CC1IF)
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC1IF;

        /**
        ignition channel 1
        */

        //trigger useful action
        set_ignition_ch1(Scheduler.targets[SCHEDULER_CH_IGN1]);

        //clean up scheduler channel
        scheduler_reset_channel(SCHEDULER_CH_IGN1);

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_ICH1_TRIG);
    }


    if( TIM5->SR & TIM_SR_CC2IF)
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC2IF;

        /**
        ignition channel 2
        */

        //trigger useful action
        set_ignition_ch1(Scheduler.targets[SCHEDULER_CH_IGN2]);

        //clean up scheduler channel
        scheduler_reset_channel(SCHEDULER_CH_IGN2);

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_ICH2_TRIG);
    }


    if( TIM5->SR & TIM_SR_CC3IF)
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC3IF;
/*
        scheduler_diag_log_event(SCHEDIAG_FCH1_TRIG);

        //disable compare irq
        TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;

        //save new state
        Scheduler.fuel1_triggered= false;


        fuel channel 1

        set_fuel_ch1(Scheduler.targets[SCHEDULER_CH_FUEL1]);
*/
    }

    if( TIM5->SR & TIM_SR_CC4IF)
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC4IF;
/*
        scheduler_diag_log_event(SCHEDIAG_FCH2_TRIG);

        //disable compare irq
        TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;

        //save new state
        Scheduler.fuel2_triggered= false;


        fuel channel 2

        set_fuel_ch2(Scheduler.targets[SCHEDULER_CH_FUEL2]);
*/
    }



    if( TIM5->SR & TIM_SR_UIF)
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_UIF;

        //this should happen only every ~9,5 h
        print(DEBUG_PORT, "scheduler wrap around");
    }





}
