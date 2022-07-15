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
#include <Tuareg_platform.h>
#include <Tuareg.h>

#include "vital_scheduler_syslog_locations.h"


volatile scheduler_mgr_t Scheduler;

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

//#define SCHEDULER_DEBUG_OUTPUT

#ifdef SCHEDULER_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // SCHEDULER_DEBUG_OUTPUT




/******************************************************************************************************************************
timer channel 1 - helper functions
******************************************************************************************************************************/

void allocate_timer_channel_1(U32 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR1= cU32max;

        //enable preload feature
        TIM5->CCMR1 |= TIM_CCMR1_OC1PE;
    }
    else
    {
        TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;
    }

    //set compare register CCR1
    TIM5->CCR1= Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC1IF;

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC1IE;
}


void reset_timer_channel_1()
{

    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC1IE;

    //disable compare preload feature
    TIM5->CCMR1 &= ~TIM_CCMR1_OC1PE;

    //set temporary compare value
    TIM5->CCR1= cU32max;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC1IF;

}


/******************************************************************************************************************************
timer channel 2 - helper functions
******************************************************************************************************************************/

void allocate_timer_channel_2(U32 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR2= cU32max;

        //enable preload feature
        TIM5->CCMR1 |= TIM_CCMR1_OC2PE;
    }
    else
    {
        TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;
    }

    //set compare register CCR2
    TIM5->CCR2= Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC2IF;

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC2IE;
}


void reset_timer_channel_2()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC2IE;

    //disable compare preload feature
    TIM5->CCMR1 &= ~TIM_CCMR1_OC2PE;

    //set temporary compare value
    TIM5->CCR2= cU32max;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC2IF;
}


/******************************************************************************************************************************
timer channel 3 - helper functions
******************************************************************************************************************************/

void allocate_timer_channel_3(U32 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR3= cU32max;

        //enable preload feature
        TIM5->CCMR2 |= TIM_CCMR2_OC3PE;
    }
    else
    {
        TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;
    }

    //set compare register CCR1
    TIM5->CCR3= Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC3IF;

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC3IE;
}

void reset_timer_channel_3()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC3IE;

    //disable compare preload feature
    TIM5->CCMR2 &= ~TIM_CCMR2_OC3PE;

    //set temporary compare value
    TIM5->CCR3= cU32max;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC3IF;

}



/******************************************************************************************************************************
timer channel 4 - helper functions
******************************************************************************************************************************/

void allocate_timer_channel_4(U32 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM5->CCR4= cU32max;

        //enable preload feature
        TIM5->CCMR2 |= TIM_CCMR2_OC4PE;
    }
    else
    {
        TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;
    }

    //set compare register CCR1
    TIM5->CCR4= Compare;

    //clear pending flag
    TIM5->SR    = (U16) ~TIM_SR_CC4IF;

    //enable irq
    TIM5->DIER |= (U16) TIM_DIER_CC4IE;
}


void reset_timer_channel_4()
{
    //disable compare irq
    TIM5->DIER &= (U16) ~TIM_DIER_CC4IE;

    //disable compare preload feature
    TIM5->CCMR2 &= ~TIM_CCMR2_OC4PE;

    //set temporary compare value
    TIM5->CCR4= cU32max;

    //clear irq pending bit
    TIM5->SR= (U16) ~TIM_SR_CC4IF;
}



/******************************************************************************************************************************
init
******************************************************************************************************************************/

void init_Vital_Scheduler()
{
    //reinit protection
    if(Scheduler.init_done == true)
    {
        return;
    }

    //clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    // clear flags
    TIM5->SR= (U16) 0;

    //set prescaler
    TIM5->PSC= (U16) (SCHEDULER_PERIOD_US * (SystemCoreClock / 1000000) - 1);

    //start timer counter
    TIM5->CR1 |= TIM_CR1_CEN;
    TIM5->EGR |= TIM_EGR_UG;

    //set up channel pointers
    Scheduler.channels[SCHEDULER_CH_IGN1].callback= set_ignition_ch1;
    Scheduler.channels[SCHEDULER_CH_IGN2].callback= set_ignition_ch2;
    Scheduler.channels[SCHEDULER_CH_FUEL1].callback= set_injector1;
    Scheduler.channels[SCHEDULER_CH_FUEL2].callback= set_injector2;
    Scheduler.channels[SCHEDULER_CH_IGN1].timer_alloc= allocate_timer_channel_1;
    Scheduler.channels[SCHEDULER_CH_IGN2].timer_alloc= allocate_timer_channel_2;
    Scheduler.channels[SCHEDULER_CH_FUEL1].timer_alloc= allocate_timer_channel_3;
    Scheduler.channels[SCHEDULER_CH_FUEL2].timer_alloc= allocate_timer_channel_4;
    Scheduler.channels[SCHEDULER_CH_IGN1].timer_reset= reset_timer_channel_1;
    Scheduler.channels[SCHEDULER_CH_IGN2].timer_reset= reset_timer_channel_2;
    Scheduler.channels[SCHEDULER_CH_FUEL1].timer_reset= reset_timer_channel_3;
    Scheduler.channels[SCHEDULER_CH_FUEL2].timer_reset= reset_timer_channel_4;

    //enable timer 5 irq (prio 2)
    NVIC_SetPriority(TIM5_IRQn, 2UL );
    NVIC_ClearPendingIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(TIM5_IRQn);

    Scheduler.init_done= true;
}




/******************************************************************************************************************************
set a scheduler channel
******************************************************************************************************************************/
void scheduler_set_channel(scheduler_channel_t Channel, volatile scheduler_activation_parameters_t * pParameters)
{
    volatile scheduler_channel_state_t * pChannelState;

    //init check
    VitalAssert(Scheduler.init_done, TID_SCHEDULER, VITALSCHED_LOC_SETCH_INITCHECK);

    //safety check - commanded channel
    VitalAssert(Channel < SCHEDULER_CH_COUNT, TID_SCHEDULER, VITALSCHED_LOC_SETCH_PARMCHECK_CH);

    //get channel reference
    pChannelState= &(Scheduler.channels[Channel]);


    /******************************************************
    parameter checks
    ******************************************************/

    //safety check - interval1_us
    VitalAssert(pParameters->interval1_us <= SCHEDULER_MAX_PERIOD_US, TID_SCHEDULER, VITALSCHED_LOC_SETCH_PARMCHECK_INT1);

    //safety check - interval2_us
    VitalAssert((pParameters->interval2_us <= SCHEDULER_MAX_PERIOD_US) || (pParameters->flags.interval2_enabled == false), TID_SCHEDULER, VITALSCHED_LOC_SETCH_PARMCHECK_INT2);

    //safety check - min interval 1
    if(pParameters->interval1_us < SCHEDULER_MIN_PERIOD_US)
    {
        pParameters->interval1_us= SCHEDULER_MIN_PERIOD_US;

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_DELAY_MININT1);
    }

    //safety check - min interval 2
    if((pParameters->flags.interval2_enabled == true) && (pParameters->interval2_us < SCHEDULER_MIN_PERIOD_US))
    {
        pParameters->interval2_us= SCHEDULER_MIN_PERIOD_US;

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_DELAY_MININT2);
    }

    //collect diagnostic information
    scheduler_diag_log_event( ((pParameters->flags.interval2_enabled == true) ? SCHEDIAG_SET_ICH1_2INT : SCHEDIAG_SET_ICH1_1INT)  + Channel );


    /******************************************************
    allocate the scheduler channel
    ******************************************************/

    //reallocation check
    if(pChannelState->flags.alloc == true)
    {
        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_ICH1_REALLOC + Channel);

        //check if the previous action shall be taken now
        if(pChannelState->parameters.flags.complete_cycle_realloc == true)
        {
            /*
            check which action shall be triggered
            - with 2 intervals enabled, check if the first interval has expired
            - with 1 interval enabled, it must always be the first action
            */
            if( (pChannelState->parameters.flags.interval2_enabled == true) && (pChannelState->flags.interval1_expired == true))
            {
                //trigger useful action 2
                pChannelState->callback(pChannelState->parameters.flags.action2_power? ACTOR_POWERED : ACTOR_UNPOWERED);
            }
            else
            {
                //trigger useful action 1
                pChannelState->callback(pChannelState->parameters.flags.action1_power? ACTOR_POWERED : ACTOR_UNPOWERED);
            }

            //collect diagnostic information
            scheduler_diag_log_event(SCHEDIAG_ICH1_REALLOC_COMPLETED + Channel);
        }
    }

    //clean the channel
    scheduler_reset_channel(Channel);

    //store the commanded actions
    pChannelState->parameters.flags.all_flags= pParameters->flags.all_flags;
    pChannelState->parameters.interval1_us= pParameters->interval1_us;
    pChannelState->parameters.interval2_us= pParameters->interval2_us;

    //set up the compare register with the first compare value
    allocate_channel(Channel, pParameters->interval1_us);
}



void scheduler_reset_channel(scheduler_channel_t Channel)
{
    volatile scheduler_channel_state_t * pChannelState;

    /**
    design warning: scheduler_reset_channel() is called inside the Fatal() function
    every VitalAssert() here may result in a loop!!!
    */


    /**
    Check if the indicated scheduler channel is valid
    */
    if(Channel < SCHEDULER_CH_COUNT)
    {
        //get channel reference
        pChannelState= &(Scheduler.channels[Channel]);

        //clear parameters
        pChannelState->parameters.flags.all_flags= 0;
        pChannelState->parameters.interval1_us= 0;
        pChannelState->parameters.interval2_us= 0;
        pChannelState->flags.all_flags= 0;
    }
    else
    {
        /**
        Dont loop here if the invalid indicated scheduler channel was the source for the fatal error
        */
        if(Tuareg.errors.fatal_error == false)
        {
            Fatal(TID_SCHEDULER, VITALSCHED_LOC_RESETCH_PARMCHECK_CH);
        }
    }


    /**
    clear timer channel

    CONSTRAINT: timer_reset() function pointer shall be dereferenced only when the scheduler has been initialised
    SOLUTION: perform an init check first
    */
    if(Scheduler.init_done == true)
    {
        pChannelState->timer_reset();
    }
    else
    {
        /**
        The vital schedulers reset function may be called uninitialized when the
        first fatal error in the system occurs earlier. E.g. config load errors
        */
        if(Tuareg.errors.fatal_error == false)
        {
            Fatal(TID_SCHEDULER, VITALSCHED_LOC_RESETCH_INITCHECK);
        }

    }

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_ICH1_RESET + Channel);
}



void allocate_channel(scheduler_channel_t Channel, U32 Delay_us)
{
    U64 compare;
    U32 now;
    bool use_preload= false;
    bool curr_cycle= false;
    volatile scheduler_channel_state_t * pChannelState;

    //get channel reference
    pChannelState= &(Scheduler.channels[Channel]);


    /******************************************************
    calculate compare value from current timer value
    ******************************************************/

    __disable_irq();

    //get current timer value
    now= TIM5->CNT;

    //compare value at delay end in ticks
    compare= now  + (Delay_us / SCHEDULER_PERIOD_US);

    __enable_irq();

    /*
    calculate the appropriate compare value and if the preload feature shall be activated
    */
    if(compare > cU32max)
    {
        /*
        the timer will wrap around until the commanded delay will expire
        the compare value to be set is the remaining amount of ticks after the timer update event
        */
        compare -= cU32max;

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

    /******************************************************
    store optional debug information
    ******************************************************/

    #ifdef SCHEDULER_DEBUG
    debug_set[debug_set_cnt].now= now;
    debug_set[debug_set_cnt].compare= compare;
    debug_set[debug_set_cnt].flags.set_curr_cycle= curr_cycle;
    debug_set[debug_set_cnt].flags.use_preload= use_preload;
    debug_set[debug_set_cnt].interval1_us= pChannelState->parameters.interval1_us;
    debug_set[debug_set_cnt].interval2_us= pChannelState->parameters.interval2_us;
    debug_set[debug_set_cnt].flags.action1_power= pChannelState->parameters.flags.action1_power;
    debug_set[debug_set_cnt].flags.action2_power= pChannelState->parameters.flags.action2_power;
    debug_set[debug_set_cnt].flags.interval2_enabled= pChannelState->parameters.flags.interval2_enabled;
    debug_set[debug_set_cnt].flags.complete_cycle_realloc= pChannelState->parameters.flags.complete_cycle_realloc;
    debug_set[debug_set_cnt].flags.realloc= pChannelState->flags.alloc;
    debug_set[debug_set_cnt].channel= Channel;


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


    /******************************************************
    allocate timer channel
    ******************************************************/

    //register the new allocation state
    pChannelState->flags.alloc= true;
    pChannelState->timer_alloc(compare, curr_cycle, use_preload);


    /******************************************************
    diagnostics
    ******************************************************/

    //collect diagnostic information
    scheduler_diag_log_event(SCHEDIAG_ICH1_ALLOC + Channel);

    if(curr_cycle == true)
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_ALLOC_CUR + Channel);
    }
    else if(use_preload == true)
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_ALLOC_PREL + Channel);
    }
    else
    {
        scheduler_diag_log_event(SCHEDIAG_ICH1_ALLOC_UPD + Channel);
    }

}


/******************************************************
vital scheduler TIM worker irq
******************************************************/
void TIM5_IRQHandler(void)
{
    volatile scheduler_channel_state_t * pChannelState;
    volatile bool trigger[SCHEDULER_CH_COUNT];
    VU32 channel;

    for(channel= 0; channel < SCHEDULER_CH_COUNT; channel++)
    {
        trigger[channel]= false;
    }


    //timer channel 1 -> ignition channel 1
    if((TIM5->SR & TIM_SR_CC1IF) && (TIM5->DIER & TIM_DIER_CC1IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC1IF;

        //channel triggered
        trigger[SCHEDULER_CH_IGN1]= true;
    }

    //timer channel 2 -> ignition channel 2
    if((TIM5->SR & TIM_SR_CC2IF) && (TIM5->DIER & TIM_DIER_CC2IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC2IF;

        //channel triggered
        trigger[SCHEDULER_CH_IGN2]= true;
    }

    //timer channel 3 -> fuel channel 1
    if((TIM5->SR & TIM_SR_CC3IF) && (TIM5->DIER & TIM_DIER_CC3IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC3IF;

        //channel triggered
        trigger[SCHEDULER_CH_FUEL1]= true;
    }

    //timer channel 4 -> fuel channel 2
    if((TIM5->SR & TIM_SR_CC4IF) && (TIM5->DIER & TIM_DIER_CC4IE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_CC4IF;

        //channel triggered
        trigger[SCHEDULER_CH_FUEL2]= true;
    }


    /******************************************************
    store optional debug information
    ******************************************************/

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


    /******************************************************
    take scheduler actions
    ******************************************************/

    for(channel= 0; channel < SCHEDULER_CH_COUNT; channel++)
    {
        //get channel reference
        pChannelState= &(Scheduler.channels[channel]);

        //check if a trigger event for this channel has occurred
        if(trigger[channel] == true)
        {
            //check which interval has expired
            if(pChannelState->flags.interval1_expired == false)
            {
                //trigger useful action 1
                pChannelState->callback(pChannelState->parameters.flags.action1_power? ACTOR_POWERED : ACTOR_UNPOWERED);

                //check if interval 2 has been commanded
                if(pChannelState->parameters.flags.interval2_enabled == true)
                {
                    //start over with interval 2
                    pChannelState->flags.interval1_expired= true;
                    pChannelState->flags.alloc= false;

                    //allocate channel again
                    allocate_channel(channel, pChannelState->parameters.interval2_us);
                }
                else
                {
                    //all done, clean up scheduler channel
                    scheduler_reset_channel(channel);
                }
            }
            else
            {
                //trigger useful action 2
                pChannelState->callback(pChannelState->parameters.flags.action2_power? ACTOR_POWERED : ACTOR_UNPOWERED);

                //all done, clean up scheduler channel
                scheduler_reset_channel(channel);
            }

            //collect diagnostic information
            scheduler_diag_log_event(SCHEDIAG_ICH1_TRIG + channel);
        }
    }


    if((TIM5->SR & TIM_SR_UIF) && (TIM5->DIER & TIM_DIER_UIE))
    {
        //clear irq pending bit
        TIM5->SR= (U16) ~TIM_SR_UIF;

        //collect diagnostic information
        scheduler_diag_log_event(SCHEDIAG_WRAP);

        #ifdef SCHEDULER_DEBUG_OUTPUT
        //this should happen only every ~9,5 h
        DebugMsg_Warning("scheduler wrap around");
        #endif // SCHEDULER_DEBUG_OUTPUT
    }

}
