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

#include "Tuareg.h"

#include "scheduler.h"
#include "ignition_hw.h"
#include "fueling_hw.h"

#include "diagnostics.h"
#include "Tuareg_errors.h"

#include "uart.h"
#include "uart_printf.h"
#include "debug_port_messages.h"

volatile scheduler_t Scheduler;


//#define SCHEDULER_DEBUG

#ifdef SCHEDULER_DEBUG
#warning scheduler debug enabled

#define DEBUG_SET_LEN 100
#define DEBUG_COMPARE_LEN 100

volatile scheduler_debug_set_t debug_set[DEBUG_SET_LEN];
volatile scheduler_debug_compare_t debug_compare[DEBUG_COMPARE_LEN];
VU32 debug_set_cnt =0;
VU32 debug_compare_cnt =0;

#endif // SCHEDULER_DEBUG

/******************************************************************************************************************************
ignition channel 1 - helper functions
******************************************************************************************************************************/

inline void scheduler_allocate_ign1(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR1= (U32) 0xFFFFFFFF;

        //enable preload feature
        TIM5->CCMR1 |= TIM_CCMR1_OC1PE;
    }
    else
    {
        TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;
    }

    //set compare register CCR1
    TIM5->CCR1= (U32) Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC1IF;

    //register the new allocation state
    Scheduler.state.ign1_alloc= true;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_ICH1_SET);

    if(CurrentCycle == true)
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_CURRC_SET);
    }
    else if(EnablePreload == true)
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_NEXTC_PRELOAD_SET);
    }
    else
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_NEXTC_UPDATE_SET);
    }

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC1IE;
}


inline void scheduler_reset_ign1()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC1IE;

    //disable compare preload feature
    TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;

    //set temporary compare value
    TIM5->CCR1= (U32) 0xFFFFFFFF;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC1IF;

    //register the new allocation state
    Scheduler.state.ign1_alloc= false;

    //store a safe action
    Scheduler.target_controls[SCHEDULER_CH_IGN1]= ACTOR_UNPOWERED;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_ICH1_RESET);
}


/******************************************************************************************************************************
ignition channel 2 - helper functions
******************************************************************************************************************************/

inline void scheduler_allocate_ign2(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR2= (U32) 0xFFFFFFFF;

        //enable preload feature
        TIM5->CCMR1 |= TIM_CCMR1_OC2PE;
    }
    else
    {
        TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;
    }

    //set compare register CCR2
    TIM5->CCR2= (U32) Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC2IF;

    //register the new allocation state
    Scheduler.state.ign2_alloc= true;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_ICH2_SET);

    if(CurrentCycle == true)
    {
        scheduler_diag_log_event(SCHEDIAG_ICH2_CURRC_SET);
    }
    else if(EnablePreload == true)
    {
        scheduler_diag_log_event(SCHEDIAG_ICH2_NEXTC_PRELOAD_SET);
    }
    else
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_NEXTC_UPDATE_SET);
    }

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC2IE;
}


inline void scheduler_reset_ign2()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC2IE;

    //disable compare preload feature
    TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;

    //set temporary compare value
    TIM5->CCR2= (U32) 0xFFFFFFFF;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC2IF;

    //register the new allocation state
    Scheduler.state.ign2_alloc= false;

    //store a safe action
    Scheduler.target_controls[SCHEDULER_CH_IGN2]= ACTOR_UNPOWERED;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_ICH2_RESET);
}


/******************************************************************************************************************************
fuel channel 1 - helper functions
******************************************************************************************************************************/

inline void scheduler_allocate_fch1(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR3= (U32) 0xFFFFFFFF;

        //enable preload feature
        TIM5->CCMR2 |= TIM_CCMR2_OC3PE;
    }
    else
    {
        TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;
    }

    //set compare register CCR1
    TIM5->CCR3= (U32) Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC3IF;

    //register the new allocation state
    Scheduler.state.fuel1_alloc= true;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_FCH1_SET);

    if(CurrentCycle == true)
    {
        scheduler_diag_log_event(SCHEDIAG_FCH1_CURRC_SET);
    }
    else if(EnablePreload == true)
    {
        scheduler_diag_log_event(SCHEDIAG_FCH1_NEXTC_PRELOAD_SET);
    }
    else
    {
        scheduler_diag_log_event(SCHEDIAG_FCH1_NEXTC_UPDATE_SET);
    }

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC3IE;
}


inline void scheduler_reset_fch1()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;

    //disable compare preload feature
    TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;

    //set temporary compare value
    TIM5->CCR3= (U32) 0xFFFFFFFF;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC3IF;

    //register the new allocation state
    Scheduler.state.fuel1_alloc= false;

    //store a safe action
    Scheduler.target_controls[SCHEDULER_CH_FUEL1]= ACTOR_UNPOWERED;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_FCH1_RESET);
}



/******************************************************************************************************************************
fuel channel 2 - helper functions
******************************************************************************************************************************/

inline void scheduler_allocate_fch2(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR4= (U32) 0xFFFFFFFF;

        //enable preload feature
        TIM5->CCMR2 |= TIM_CCMR2_OC4PE;
    }
    else
    {
        TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;
    }

    //set compare register CCR1
    TIM5->CCR4= (U32) Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC4IF;

    //register the new allocation state
    Scheduler.state.fuel2_alloc= true;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_FCH2_SET);

    if(CurrentCycle == true)
    {
        scheduler_diag_log_event(SCHEDIAG_FCH2_CURRC_SET);
    }
    else if(EnablePreload == true)
    {
        scheduler_diag_log_event(SCHEDIAG_FCH2_NEXTC_PRELOAD_SET);
    }
    else
    {
        scheduler_diag_log_event(SCHEDIAG_FCH2_NEXTC_UPDATE_SET);
    }

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC4IE;
}


inline void scheduler_reset_fch2()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;

    //disable compare preload feature
    TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;

    //set temporary compare value
    TIM5->CCR4= (U32) 0xFFFFFFFF;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC4IF;

    //register the new allocation state
    Scheduler.state.fuel2_alloc= false;

    //store a safe action
    Scheduler.target_controls[SCHEDULER_CH_FUEL2]= ACTOR_UNPOWERED;

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_FCH2_RESET);
}



/******************************************************************************************************************************
init
******************************************************************************************************************************/

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

    //reset channels

    //enable timer 5 irq (prio 2)
    NVIC_SetPriority(TIM5_IRQn, 2UL );
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(TIM5_IRQn);
}




/******************************************************************************************************************************
set a scheduler channel
******************************************************************************************************************************/
void scheduler_set_channel(scheduler_channel_t Channel, actor_control_t Controls, VU32 Delay_us, volatile bool Complete_on_realloc)
{
/// TODO (oli#5#): implement syslog for scheduler
    VU64 compare;
    VU32 now;
    volatile bool use_preload= false;
    volatile bool curr_cycle= false;

    /******************************************************
    parameter checks
    ******************************************************/

    //safety check - requested channel
    Assert(Channel < SCHEDULER_CH_COUNT, TID_SCHEDULER, 1);

    //safety check - clip delay
    if(Delay_us > SCHEDULER_MAX_PERIOD_US)
    {
        /*
        Delay_us= SCHEDULER_MAX_PERIOD_US;
        */

        scheduler_diag_log_event(SCHEDIAG_DELAY_CLIPPED);

        return;
    }

    if(Delay_us < SCHEDULER_MIN_PERIOD_US)
    {
        /**
        take immediate action - no scheduler allocation actually
        */

        scheduler_diag_log_event(SCHEDIAG_DELAY_BYPASS);

        switch(Channel)
        {
            case SCHEDULER_CH_IGN1:
                set_ignition_ch1(Controls);
                break;

            case SCHEDULER_CH_IGN2:
                set_ignition_ch2(Controls);
                break;

            case SCHEDULER_CH_FUEL1:
                set_injector1(Controls);
                break;

            case SCHEDULER_CH_FUEL2:
                set_injector2(Controls);
                break;

            default:
                break;
        }

        //no scheduler allocation
        return;
    }


    /******************************************************
    calculate compare value
    ******************************************************/

    __disable_irq();

    //get current timer value
    now= TIM5->CNT;

    //compare value at delay end in ticks
    compare= now  + (Delay_us / SCHEDULER_PERIOD_US);

    #ifdef SCHEDULER_DEBUG
    debug_set[debug_set_cnt].now= now;
    debug_set[debug_set_cnt].compare= compare;
    debug_set[debug_set_cnt].param_delay_us= Delay_us;
    debug_set[debug_set_cnt].flags.target_state_powered= (Controls == ACTOR_POWERED)? true : false;
    debug_set[debug_set_cnt].flags.param_complete_realloc= Complete_on_realloc;
    #endif // SCHEDULER_DEBUG

    /*
    calculate the appropriate compare value and if the preload feature shall be activated
    */
    if(compare > 0xFFFFFFFF)
    {
        /*
        the timer will wrap around until the commanded delay will expire
        the compare value to be set is the remaining amount of ticks after the timer update event
        */
        compare -= 0xFFFFFFFF;

        /*
        check if setting the new timer compare value would "short circuit" the timer update event
        then we shall the use the preload feature
        -> preload function will load new compare value after update event
        */
        use_preload= (compare > now)? true : false;
    }
    else
    {
        curr_cycle= true;
    }

    __enable_irq();

    #ifdef SCHEDULER_DEBUG
    debug_set[debug_set_cnt].flags.set_next_cycle_preload= use_preload;
    debug_set[debug_set_cnt].flags.set_curr_cycle= curr_cycle;
    #endif // SCHEDULER_DEBUG


    /******************************************************
    allocate the scheduler channel
    ******************************************************/

    switch(Channel)
    {
        case SCHEDULER_CH_IGN1:

            #ifdef SCHEDULER_DEBUG
            debug_set[debug_set_cnt].flags.param_ch_ign1= true;
            #endif // SCHEDULER_DEBUG

            //reallocation check
            if(Scheduler.state.ign1_alloc == true)
            {
                #ifdef SCHEDULER_DEBUG
                debug_set[debug_set_cnt].flags.reactivated= true;
                #endif // SCHEDULER_DEBUG

                //collect diagnostic information
                scheduler_diag_log_event(SCHEDIAG_ICH1_RETRIGD);

                //check if the previous action shall be taken now
                if(Complete_on_realloc == true)
                {
                    //complete the last scheduler cycle with its commanded action
                    set_ignition_ch1(Scheduler.target_controls[Channel]);
                }

            }

            //clean the channel
            scheduler_reset_ign1();

            //store the desired action at delay end
            Scheduler.target_controls[Channel]= Controls;

            //set up the compare register with the compare value
            scheduler_allocate_ign1((U32) compare, curr_cycle, use_preload);

            break;


        case SCHEDULER_CH_IGN2:

            #ifdef SCHEDULER_DEBUG
            debug_set[debug_set_cnt].flags.param_ch_ign2= true;
            #endif // SCHEDULER_DEBUG

            //reallocation check
            if(Scheduler.state.ign2_alloc == true)
            {
                #ifdef SCHEDULER_DEBUG
                debug_set[debug_set_cnt].flags.reactivated= true;
                #endif // SCHEDULER_DEBUG

                //collect diagnostic information
                scheduler_diag_log_event(SCHEDIAG_ICH2_RETRIGD);

                //check if the previous action shall be taken now
                if(Complete_on_realloc == true)
                {
                    //complete the last scheduler cycle with its commanded action
                    set_ignition_ch2(Scheduler.target_controls[Channel]);
                }

            }

            //clean the channel
            scheduler_reset_ign2();

            //store the desired action at delay end
            Scheduler.target_controls[Channel]= Controls;

            //set up the compare register with the compare value
            scheduler_allocate_ign2((U32) compare, curr_cycle, use_preload);
            break;



        case SCHEDULER_CH_FUEL1:

            #ifdef SCHEDULER_DEBUG
            debug_set[debug_set_cnt].flags.param_ch_fuel1= true;
            #endif // SCHEDULER_DEBUG

            //reallocation check
            if(Scheduler.state.fuel1_alloc == true)
            {
                #ifdef SCHEDULER_DEBUG
                debug_set[debug_set_cnt].flags.reactivated= true;
                #endif // SCHEDULER_DEBUG

                //collect diagnostic information
                scheduler_diag_log_event(SCHEDIAG_FCH1_RETRIGD);

                //check if the previous action shall be taken now
                if(Complete_on_realloc == true)
                {
                    //complete the last scheduler cycle with its commanded action
                    set_injector1(Scheduler.target_controls[Channel]);
                }

            }

            //clean the channel
            scheduler_reset_fch1();

            //store the desired action at delay end
            Scheduler.target_controls[Channel]= Controls;

            //set up the compare register with the compare value
            scheduler_allocate_fch1((U32) compare, curr_cycle, use_preload);

            break;



        case SCHEDULER_CH_FUEL2:

            #ifdef SCHEDULER_DEBUG
            debug_set[debug_set_cnt].flags.param_ch_fuel2= true;
            #endif // SCHEDULER_DEBUG

            //reallocation check
            if(Scheduler.state.fuel2_alloc == true)
            {
                #ifdef SCHEDULER_DEBUG
                debug_set[debug_set_cnt].flags.reactivated= true;
                #endif // SCHEDULER_DEBUG

                //collect diagnostic information
                scheduler_diag_log_event(SCHEDIAG_FCH2_RETRIGD);

                //check if the previous action shall be taken now
                if(Complete_on_realloc == true)
                {
                    //complete the last scheduler cycle with its commanded action
                    set_injector2(Scheduler.target_controls[Channel]);
                }

            }

            //clean the channel
            scheduler_reset_fch2();

            //store the desired action at delay end
            Scheduler.target_controls[Channel]= Controls;

            //set up the compare register with the compare value
            scheduler_allocate_fch2((U32) compare, curr_cycle, use_preload);
            break;

    default:
        break;

    }


    #ifdef SCHEDULER_DEBUG
    if(debug_set_cnt < DEBUG_SET_LEN -1)
    {
        debug_set_cnt++;
    }
    else
    {
        DebugMsg_Warning("scheduler capture ready");
        return;
    }
    #endif // SCHEDULER_DEBUG

}







void TIM5_IRQHandler(void)
{
    #ifdef SCHEDULER_DEBUG
    debug_compare[debug_compare_cnt].CNT= TIM5->CNT;

    debug_compare[debug_compare_cnt].flags.sr_comp1= (TIM5->SR & TIM_SR_CC1IF)? true : false;
    debug_compare[debug_compare_cnt].flags.sr_comp2= (TIM5->SR & TIM_SR_CC2IF)? true : false;
    debug_compare[debug_compare_cnt].flags.sr_comp3= (TIM5->SR & TIM_SR_CC3IF)? true : false;
    debug_compare[debug_compare_cnt].flags.sr_comp4= (TIM5->SR & TIM_SR_CC4IF)? true : false;
    debug_compare[debug_compare_cnt].flags.sr_update= (TIM5->SR & TIM_SR_UIF)? true : false;

    debug_compare[debug_compare_cnt].flags.dier_comp1= (TIM5->DIER & TIM_DIER_CC1IE)? true : false;
    debug_compare[debug_compare_cnt].flags.dier_comp2= (TIM5->DIER & TIM_DIER_CC2IE)? true : false;
    debug_compare[debug_compare_cnt].flags.dier_comp3= (TIM5->DIER & TIM_DIER_CC3IE)? true : false;
    debug_compare[debug_compare_cnt].flags.dier_comp4= (TIM5->DIER & TIM_DIER_CC4IE)? true : false;
    debug_compare[debug_compare_cnt].flags.dier_update= (TIM5->DIER & TIM_DIER_UIE)? true : false;

    debug_compare[debug_compare_cnt].COMP1= TIM5->CCR1;
    debug_compare[debug_compare_cnt].COMP2= TIM5->CCR2;
    debug_compare[debug_compare_cnt].COMP3= TIM5->CCR3;
    debug_compare[debug_compare_cnt].COMP4= TIM5->CCR4;


    if(debug_compare_cnt < DEBUG_COMPARE_LEN -1)
    {
        debug_compare_cnt++;
    }
    else
    {
        DebugMsg_Warning("scheduler debug");
        return;
    }

    #endif // SCHEDULER_DEBUG

    if((TIM5->SR & TIM_SR_CC1IF) && (TIM5->DIER & TIM_DIER_CC1IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC1IF;

        /**
        ignition channel 1
        */

        //trigger useful action
        set_ignition_ch1(Scheduler.target_controls[SCHEDULER_CH_IGN1]);

        //clean up scheduler channel
        scheduler_reset_ign1();

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_ICH1_TRIG);
    }


    if((TIM5->SR & TIM_SR_CC2IF) && (TIM5->DIER & TIM_DIER_CC2IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC2IF;

        /**
        ignition channel 2
        */

        //trigger useful action
        set_ignition_ch2(Scheduler.target_controls[SCHEDULER_CH_IGN2]);

        //clean up scheduler channel
        scheduler_reset_ign2();

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_ICH2_TRIG);
    }


    if((TIM5->SR & TIM_SR_CC3IF) && (TIM5->DIER & TIM_DIER_CC3IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC3IF;

        /**
        fuel channel 1
        */

        //trigger useful action
        set_injector1(Scheduler.target_controls[SCHEDULER_CH_FUEL1]);

        //clean up scheduler channel
        scheduler_reset_fch1();

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_FCH1_TRIG);
    }


    if((TIM5->SR & TIM_SR_CC4IF) && (TIM5->DIER & TIM_DIER_CC4IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC4IF;

        /**
        fuel channel 2
        */

        //trigger useful action
        set_injector2(Scheduler.target_controls[SCHEDULER_CH_FUEL2]);

        //clean up scheduler channel
        scheduler_reset_fch2();

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_FCH2_TRIG);
    }


    if((TIM5->SR & TIM_SR_UIF) && (TIM5->DIER & TIM_DIER_UIE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_UIF;

        //this should happen only every ~9,5 h
        DebugMsg_Warning("scheduler wrap around");
    }

}
