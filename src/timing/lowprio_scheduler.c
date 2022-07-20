/**
this scheduler helps to control low priority system actions (e.g. blink user lamp, control dash, ...)

it uses the 4 compare channels on 16 bit timer 11 for this

the 4 scheduler channels can be allocated freely to callback functions that can take an output_pin_t as parameter
*/
#include <Tuareg_platform.h>
#include <Tuareg.h>

#include "lowprio_scheduler_syslog_locations.h"

#include "dash_hw.h"



#ifndef LOWPRIOSCHEDULER_WIP


volatile lowprio_scheduler_mgr_t Lowprio_Scheduler;


static void safe_callback(actor_control_t dummy)
{

}


/******************************************************************************************************************************
timer channel 1 - helper functions
******************************************************************************************************************************/

void allocate_lowprio_timer_channel_1(U16 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM1->CCR1= cU16max;

        //enable preload feature
        TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    }
    else
    {
        TIM1->CCMR1 &= ~TIM_CCMR1_OC1PE;
    }

    //set compare register CCR1
    TIM1->CCR1= Compare;

    //clear pending flag
    TIM1->SR    = (U16) ~TIM_SR_CC1IF;

    //enable irq
    TIM1->DIER |= (U16) TIM_DIER_CC1IE;
}


void reset_lowprio_timer_channel_1()
{

    //disable compare irq
    TIM1->DIER &= (U16) ~TIM_DIER_CC1IE;

    //disable compare preload feature
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1PE;

    //set temporary compare value
    TIM1->CCR1= cU16max;

    //clear irq pending bit
    TIM1->SR= (U16) ~TIM_SR_CC1IF;

}


/******************************************************************************************************************************
timer channel 2 - helper functions
******************************************************************************************************************************/

void allocate_lowprio_timer_channel_2(U16 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM1->CCR2= cU16max;

        //enable preload feature
        TIM1->CCMR1 |= TIM_CCMR1_OC2PE;
    }
    else
    {
        TIM1->CCMR1 &= ~TIM_CCMR1_OC2PE;
    }

    //set compare register CCR2
    TIM1->CCR2= Compare;

    //clear pending flag
    TIM1->SR    = (U16) ~TIM_SR_CC2IF;

    //enable irq
    TIM1->DIER |= (U16) TIM_DIER_CC2IE;
}


void reset_lowprio_timer_channel_2()
{
    //disable compare irq
    TIM1->DIER &= (U16) ~TIM_DIER_CC2IE;

    //disable compare preload feature
    TIM1->CCMR1 &= ~TIM_CCMR1_OC2PE;

    //set temporary compare value
    TIM1->CCR2= cU16max;

    //clear irq pending bit
    TIM1->SR= (U16) ~TIM_SR_CC2IF;
}


/******************************************************************************************************************************
timer channel 3 - helper functions
******************************************************************************************************************************/

void allocate_lowprio_timer_channel_3(U16 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM1->CCR3= cU16max;

        //enable preload feature
        TIM1->CCMR2 |= TIM_CCMR2_OC3PE;
    }
    else
    {
        TIM1->CCMR2 &= ~TIM_CCMR2_OC3PE;
    }

    //set compare register CCR1
    TIM1->CCR3= Compare;

    //clear pending flag
    TIM1->SR    = (U16) ~TIM_SR_CC3IF;

    //enable irq
    TIM1->DIER |= (U16) TIM_DIER_CC3IE;
}

void reset_lowprio_timer_channel_3()
{
    //disable compare irq
    TIM1->DIER &= (U16) ~TIM_DIER_CC3IE;

    //disable compare preload feature
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3PE;

    //set temporary compare value
    TIM1->CCR3= cU16max;

    //clear irq pending bit
    TIM1->SR= (U16) ~TIM_SR_CC3IF;

}



/******************************************************************************************************************************
timer channel 4 - helper functions
******************************************************************************************************************************/

void allocate_lowprio_timer_channel_4(U16 Compare, bool CurrentCycle, bool EnablePreload)
{
    //check if the preload feature has been requested
    if(EnablePreload == true)
    {
        //use a temporary compare value that will not trigger any compare events
        TIM1->CCR4= cU16max;

        //enable preload feature
        TIM1->CCMR2 |= TIM_CCMR2_OC4PE;
    }
    else
    {
        TIM1->CCMR2 &= ~TIM_CCMR2_OC4PE;
    }

    //set compare register CCR1
    TIM1->CCR4= Compare;

    //clear pending flag
    TIM1->SR    = (U16) ~TIM_SR_CC4IF;

    //enable irq
    TIM1->DIER |= (U16) TIM_DIER_CC4IE;
}


void reset_lowprio_timer_channel_4()
{
    //disable compare irq
    TIM1->DIER &= (U16) ~TIM_DIER_CC4IE;

    //disable compare preload feature
    TIM1->CCMR2 &= ~TIM_CCMR2_OC4PE;

    //set temporary compare value
    TIM1->CCR4= cU16max;

    //clear irq pending bit
    TIM1->SR= (U16) ~TIM_SR_CC4IF;
}


/******************************************************************************************************************************
helper functions
******************************************************************************************************************************/
void lowprio_scheduler_reset_channel(lowprio_scheduler_channel_t Channel)
{
    volatile lowprio_scheduler_channel_state_t * pChannelState;

    //init check
    VitalAssert(Lowprio_Scheduler.init_done, TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_RESETCH_INITCHECK);

    //safety check - requested channel
    VitalAssert(Channel < LOWPRIO_CH_COUNT, TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_RESETCH_PARMCHECK_CH);

    //get channel reference
    pChannelState= &(Lowprio_Scheduler.channels[Channel]);

    //clear parameters
    pChannelState->parameters.flags.all_flags= 0;
    pChannelState->parameters.intervals_us[INTERVAL_A]= 0;
    pChannelState->parameters.intervals_us[INTERVAL_B]= 0;
    pChannelState->parameters.intervals_us[INTERVAL_PAUSE]= 0;
    pChannelState->parameters.sequence_length= 0;
    pChannelState->parameters.cycles= 0;
    pChannelState->sequence_counter= 0;
    pChannelState->cycle_counter= 0;

    //clear timer channel
    pChannelState->timer_reset();
}



void lowprio_scheduler_allocate_channel(lowprio_scheduler_channel_t Channel, lowprio_scheduler_intervals_t Interval)
{
    U32 compare, interval_us, now;
    bool use_preload= false;
    bool curr_cycle= false;
    volatile lowprio_scheduler_channel_state_t * pChannelState;

    //get channel reference
    pChannelState= &(Lowprio_Scheduler.channels[Channel]);

    //get interval length
    interval_us= pChannelState->parameters.intervals_us[Interval];

    //store allocated interval
    pChannelState->allocated_interval= Interval;

    /******************************************************
    calculate compare value from current timer value
    ******************************************************/

    __disable_irq();

    //get current timer value
    now= TIM1->CNT;

    //compare value at delay end in ticks
    compare= now  + (interval_us / LOWPRIO_SCHEDULER_PERIOD_US);

    __enable_irq();

    /*
    calculate the appropriate compare value and if the preload feature shall be activated
    */
    if(compare > cU16max)
    {
        /*
        the timer will wrap around until the commanded delay will expire
        the compare value to be set is the remaining amount of ticks after the timer update event
        */
        compare -= cU16max;

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
    allocate timer channel
    ******************************************************/

    //register the new allocation state
    pChannelState->timer_alloc(compare, curr_cycle, use_preload);
}


/******************************************************************************************************************************
init
******************************************************************************************************************************/

void init_Lowprio_Scheduler()
{
    //reinit protection
    if(Lowprio_Scheduler.init_done == true)
    {
        return;
    }

    //clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // clear flags
    TIM1->SR= (U16) 0;

    //set prescaler
    TIM1->PSC= (U16) LOWPRIO_SCHEDULER_PERIOD_US * (SystemCoreClock / 1000000) - 1;

    //start timer counter
    TIM1->CR1 |= TIM_CR1_CEN;
    //TIM1->EGR |= TIM_EGR_UG;

    //set up channel pointers
    Lowprio_Scheduler.channels[LOWPRIO_CH1].callback= safe_callback;
    Lowprio_Scheduler.channels[LOWPRIO_CH2].callback= safe_callback;
    Lowprio_Scheduler.channels[LOWPRIO_CH3].callback= safe_callback;
    Lowprio_Scheduler.channels[LOWPRIO_CH_TACH].callback= set_tachometer_hw;

    Lowprio_Scheduler.channels[LOWPRIO_CH1].timer_alloc= allocate_lowprio_timer_channel_1;
    Lowprio_Scheduler.channels[LOWPRIO_CH2].timer_alloc= allocate_lowprio_timer_channel_2;
    Lowprio_Scheduler.channels[LOWPRIO_CH3].timer_alloc= allocate_lowprio_timer_channel_3;
    Lowprio_Scheduler.channels[LOWPRIO_CH_TACH].timer_alloc= allocate_lowprio_timer_channel_4;

    Lowprio_Scheduler.channels[LOWPRIO_CH1].timer_reset= reset_lowprio_timer_channel_1;
    Lowprio_Scheduler.channels[LOWPRIO_CH2].timer_reset= reset_lowprio_timer_channel_2;
    Lowprio_Scheduler.channels[LOWPRIO_CH3].timer_reset= reset_lowprio_timer_channel_3;
    Lowprio_Scheduler.channels[LOWPRIO_CH_TACH].timer_reset= reset_lowprio_timer_channel_4;

    //enable timer 1 compare irq (prio 15)
    NVIC_SetPriority(TIM1_CC_IRQn, 15UL );
    NVIC_ClearPendingIRQ(TIM1_CC_IRQn);
    NVIC_EnableIRQ(TIM1_CC_IRQn);

    Lowprio_Scheduler.init_done= true;
}


/******************************************************************************************************************************
lowprio scheduler interface

scheduler operating modes:
- one shot mode: wait until interval_1 has expired, take action_1, reset channel
--> power_after_interval_A must not be enabled
--> rationale?

    activation: interval_2_enabled= false, interval_3_enabled= false
- pwm mode: take action_1 at the end of interval_1 and !action_1 at the end of interval_2; runs forever if free_running= true or for the amount of "cycles"
    activation: interval_1,2_enabled=true; interval_3_enabled= false
- sequence mode: like pwm mode, but inserts interval_3 after "sequence_length" repetitions of interval_1/interval_2


******************************************************************************************************************************/
void lowprio_scheduler_set_channel(lowprio_scheduler_channel_t Channel, lowprio_scheduler_activation_parameters_t * pParameters)
{
    volatile lowprio_scheduler_channel_state_t * pChannelState;

    //init check
    VitalAssert(Lowprio_Scheduler.init_done, TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_INITCHECK);

    //safety check - commanded channel
    VitalAssert(Channel < LOWPRIO_CH_COUNT, TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_CH);

    //get channel reference
    pChannelState= &(Lowprio_Scheduler.channels[Channel]);


    /******************************************************
    parameter checks
    ******************************************************/

    //safety check - interval1_us
    VitalAssert(pParameters->intervals_us[INTERVAL_A] <= LOWPRIO_SCHEDULER_MAX_PERIOD_US, TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_INTA);

    //safety check - interval2_us
    VitalAssert((pParameters->intervals_us[INTERVAL_B] <= LOWPRIO_SCHEDULER_MAX_PERIOD_US) || (pParameters->flags.interval_B_enabled == false), TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_INTB);

    //safety check - interval3_us
    VitalAssert((pParameters->intervals_us[INTERVAL_PAUSE] <= LOWPRIO_SCHEDULER_MAX_PERIOD_US) || (pParameters->flags.interval_Pause_enabled == false), TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_INTP);

    //safety check - cycles
    VitalAssert( (pParameters->flags.free_running == true) || (pParameters->cycles > 0), TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_CYCLES);

    //safety check - sequence length
    VitalAssert( (pParameters->flags.interval_Pause_enabled == false) || (pParameters->sequence_length > 0), TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_SEQLEN);

    //safety check - polarity in one shot mode
   // VitalAssert( (pParameters->flags.interval_B_enabled == true) || (pParameters->flags.power_after_interval_A == false), TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_SETCH_PARMCHECK_POLARITY);


    //safety check - min interval 1
    if(pParameters->intervals_us[INTERVAL_A] < LOWPRIO_SCHEDULER_MIN_PERIOD_US)
    {
        pParameters->intervals_us[INTERVAL_A]= LOWPRIO_SCHEDULER_MIN_PERIOD_US;
    }

    //safety check - min interval 2
    if((pParameters->flags.interval_B_enabled == true) && (pParameters->intervals_us[INTERVAL_B] < LOWPRIO_SCHEDULER_MIN_PERIOD_US))
    {
        pParameters->intervals_us[INTERVAL_B]= LOWPRIO_SCHEDULER_MIN_PERIOD_US;
    }

    //safety check - min interval 3
    if((pParameters->flags.interval_Pause_enabled == true) && (pParameters->intervals_us[INTERVAL_PAUSE] < LOWPRIO_SCHEDULER_MIN_PERIOD_US))
    {
        pParameters->intervals_us[INTERVAL_PAUSE]= LOWPRIO_SCHEDULER_MIN_PERIOD_US;
    }

    /******************************************************
    allocate the scheduler channel
    ******************************************************/

    //clean the channel parameters
    lowprio_scheduler_reset_channel(Channel);

    //store the commanded actions
    pChannelState->parameters.flags.all_flags= pParameters->flags.all_flags;
    pChannelState->parameters.intervals_us[INTERVAL_A]= pParameters->intervals_us[INTERVAL_A];
    pChannelState->parameters.intervals_us[INTERVAL_B]= pParameters->intervals_us[INTERVAL_B];
    pChannelState->parameters.intervals_us[INTERVAL_PAUSE]= pParameters->intervals_us[INTERVAL_PAUSE];
    pChannelState->parameters.sequence_length= pParameters->sequence_length;
    pChannelState->parameters.cycles= pParameters->cycles;

    //init counters
    pChannelState->cycle_counter= pChannelState->parameters.cycles;
    pChannelState->sequence_counter= pChannelState->parameters.sequence_length;

    //allocate the lowprio scheduler with the first interval
    lowprio_scheduler_allocate_channel(Channel, INTERVAL_A);
}



void TIM1_UP_TIM10_IRQHandler(void)
{
    //if((TIM1->SR & TIM_SR_UIF) && (TIM1->DIER & TIM_DIER_UIE))
    if(TIM1->SR & TIM_SR_UIF)
    {
        //clear irq pending bit
        TIM1->SR= (U16) ~TIM_SR_UIF;
    }
}



/******************************************************
lowprio scheduler timer worker irq
******************************************************/
void TIM1_CC_IRQHandler(void)
{
    volatile lowprio_scheduler_channel_state_t * pChannelState;
    bool trigger[LOWPRIO_CH_COUNT];
    U32 channel;

    for(channel= 0; channel < LOWPRIO_CH_COUNT; channel++)
    {
        trigger[channel]= false;
    }

    //timer channel 1
    if((TIM1->SR & TIM_SR_CC1IF) && (TIM1->DIER & TIM_DIER_CC1IE))
    {
        //clear irq pending bit
        TIM1->SR= (U16) ~TIM_SR_CC1IF;

        //channel triggered
        trigger[LOWPRIO_CH1]= true;
    }

    //timer channel 2
    if((TIM1->SR & TIM_SR_CC2IF) && (TIM1->DIER & TIM_DIER_CC2IE))
    {
        //clear irq pending bit
        TIM1->SR= (U16) ~TIM_SR_CC2IF;

        //channel triggered
        trigger[LOWPRIO_CH2]= true;
    }

    //timer channel 3
    if((TIM1->SR & TIM_SR_CC3IF) && (TIM1->DIER & TIM_DIER_CC3IE))
    {
        //clear irq pending bit
        TIM1->SR= (U16) ~TIM_SR_CC3IF;

        //channel triggered
        trigger[LOWPRIO_CH3]= true;
    }

    //timer channel 4
    if((TIM1->SR & TIM_SR_CC4IF) && (TIM1->DIER & TIM_DIER_CC4IE))
    {
        //clear irq pending bit
        TIM1->SR= (U16) ~TIM_SR_CC4IF;

        //channel triggered
        trigger[LOWPRIO_CH_TACH]= true;
    }


    /******************************************************
    take scheduler actions
    ******************************************************/

    for(channel= 0; channel < LOWPRIO_CH_COUNT; channel++)
    {
        //get channel reference
        pChannelState= &(Lowprio_Scheduler.channels[channel]);

        //check if a trigger event for this channel has occurred
        if(trigger[channel] == true)
        {
            //check which interval has expired
            switch(pChannelState->allocated_interval)
            {

            case INTERVAL_PAUSE:

                //check if a cycle length has to be observed
                if((pChannelState->parameters.flags.free_running == false) && (pChannelState->cycle_counter == 0))
                {
                    //the commanded amount of cycles has been executed -> finish
                    lowprio_scheduler_reset_channel(channel);
                }
                else
                {
                    /**
                    go on with interval "A"
                    */

                    //trigger action "pause" (action "pause" := action "B") (action "B" := !action "A")
                    pChannelState->callback(pChannelState->parameters.flags.power_after_interval_A? ACTOR_UNPOWERED : ACTOR_POWERED);

                    //begin next sequence
                    pChannelState->sequence_counter= pChannelState->parameters.sequence_length;

                    //one cycle in sequence mode has been completed
                    pChannelState->cycle_counter -= 1;

                    //allocate channel with "A" interval
                    lowprio_scheduler_allocate_channel(channel, INTERVAL_A);
                }

                break;


            case INTERVAL_A:

                    //check if pwm or sequence mode has been commanded (interval "B" enabled)
                    if(pChannelState->parameters.flags.interval_B_enabled == true)
                    {
                        //trigger action "A"
                        pChannelState->callback(pChannelState->parameters.flags.power_after_interval_A? ACTOR_POWERED : ACTOR_UNPOWERED);

                        //allocate channel with "B" interval
                        lowprio_scheduler_allocate_channel(channel, INTERVAL_B);
                    }
                    else
                    {
                        ///one shot mode

                        //trigger action "A"
                        pChannelState->callback(pChannelState->parameters.flags.power_after_interval_A? ACTOR_POWERED : ACTOR_UNPOWERED);

                        //finish
                        lowprio_scheduler_reset_channel(channel);
                    }

                    break;


            case INTERVAL_B:

                    /**
                    interval "B" has expired
                    - in one shot mode -> should not be reached, error!
                    - in pwm and sequence mode -> take action_1, load interval_2;
                    - end for pwm mode if not free running and cycles expired; count cycles if left
                    - if free_running= false and reset channel when all cycles have been executed
                    */


                    //check if pwm mode has been commanded
                    if((pChannelState->parameters.flags.interval_B_enabled == true) && (pChannelState->parameters.flags.interval_Pause_enabled == false))
                    {
                        /**
                        pwm mode: "A" follows "B" interval, if free running or if cycles left
                        */

                        //check if a cycle length has to be observed
                        if((pChannelState->parameters.flags.free_running == false) && (pChannelState->cycle_counter == 0))
                        {
                            ///the commanded amount of cycles has been executed

                            //unpower actor
                            pChannelState->callback(ACTOR_UNPOWERED);

                            //finish
                            lowprio_scheduler_reset_channel(channel);
                        }
                        else
                        {
                            //trigger action B (action B := !action A)
                            pChannelState->callback(pChannelState->parameters.flags.power_after_interval_A? ACTOR_UNPOWERED : ACTOR_POWERED);

                            //one cycle in pwm mode has been completed
                            pChannelState->cycle_counter -= 1;

                            //allocate channel with "A" interval
                            lowprio_scheduler_allocate_channel(channel, INTERVAL_A);
                        }
                    }
                    else if((pChannelState->parameters.flags.interval_B_enabled == true) && (pChannelState->parameters.flags.interval_Pause_enabled == true))
                    {
                        /**
                        sequence mode: "Pause" follows interval "B", if sequence length has expired; else "A"
                        */

                        //check if sequence length has expired
                        if(pChannelState->sequence_counter == 0)
                        {
                            //allocate channel with "Pause" interval
                            lowprio_scheduler_allocate_channel(channel, INTERVAL_PAUSE);
                        }
                        else
                        {
                            //one sequence step has been completed
                            pChannelState->sequence_counter -= 1;

                            //trigger action B (action B := !action A)
                            pChannelState->callback(pChannelState->parameters.flags.power_after_interval_A? ACTOR_UNPOWERED : ACTOR_POWERED);

                            //allocate channel with "A" interval
                            lowprio_scheduler_allocate_channel(channel, INTERVAL_A);
                        }
                    }
                    else
                    {
                        Fatal(TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_IRQ_INVAL_MODE);
                    }

                    break;

            default:
                    Fatal(TID_LOWPRIO_SCHEDULER, LPSCHED_LOC_IRQ_INVAL_INTERVAL);
                    break;

            }








        }
    }



    //if((TIM1->SR & TIM_SR_UIF) && (TIM1->DIER & TIM_DIER_UIE))
    if(TIM1->SR & TIM_SR_UIF)
    {
        //clear irq pending bit
        TIM1->SR= (U16) ~TIM_SR_UIF;
    }

}

#endif
