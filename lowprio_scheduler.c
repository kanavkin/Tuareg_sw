/**
this scheduler helps to control low priority system actions (e.g. blink user lamp, control dash, ...)

it uses the 4 compare channels on 16 bit timer 11 for this

the 4 scheduler channels can be allocated freely to callback functions that can take an output_pin_t as parameter
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "lowprio_scheduler.h"
#include "ignition_hw.h"
#include "fuel_hw.h"

volatile lowprio_scheduler_t Lowprio_Scheduler;


static void safe_callback(output_pin_t dummy)
{

}



void init_lowprio_scheduler()
{
    //internals
    Lowprio_Scheduler.toggle_ctrl =0;
    Lowprio_Scheduler.ch1_callback= safe_callback;
    Lowprio_Scheduler.ch2_callback= safe_callback;
    Lowprio_Scheduler.ch3_callback= safe_callback;
    Lowprio_Scheduler.ch4_callback= safe_callback;


    //clock
    RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;

    // clear flags
    TIM11->SR= (U16) 0;

    //set prescaler
    TIM11->PSC= (U16) LOWPRIO_SCHEDULER_PERIOD_US * (SystemCoreClock / 1000000) - 1;

    //start timer counter
    TIM11->CR1 |= TIM_CR1_CEN;
    TIM11->EGR |= TIM_EGR_UG;

    //enable timer 11 irq (prio 2)
    NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 15UL );
    NVIC_ClearPendingIRQ(TIM1_TRG_COM_TIM11_IRQn);
    NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}



/**
activates sequence mode on the commanded lowprio_scheduler channel (CH1/CH2 only!) (free running forever)

the lowprio_scheduler will automatically reload with alternating delay1/delay_2 seq_len times and request toggling the desired pin

invoke lowprio_scheduler_reset_channel() to end toggle mode

the callback function must support "TOGGLE" as parameter
*/
void lowprio_scheduler_seqmode_channel(lowprio_scheduler_channel_t target_ch, void (* callback_funct)(output_pin_t), U32 delay1_us, U32 delay2_us, U32 delay3_us, U32 seq_length)
{
    switch(target_ch)
    {
        case LOWPRIO_CH1:

            //save command
            Lowprio_Scheduler.ch1_delay1_us= delay1_us;
            Lowprio_Scheduler.ch1_delay2_us= delay2_us;
            Lowprio_Scheduler.ch1_delay3_us= delay3_us;

            //save sequencer parameters
            Lowprio_Scheduler.ch1_sequence_length= seq_length;
            Lowprio_Scheduler.ch1_toggle_counter= seq_length;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH1_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH1_SEQUENCE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH1_CYCLE_MASK);


            break;

        case LOWPRIO_CH2:

            //save command
            Lowprio_Scheduler.ch2_delay1_us= delay1_us;
            Lowprio_Scheduler.ch2_delay2_us= delay2_us;
            Lowprio_Scheduler.ch2_delay3_us= delay3_us;

            //save sequencer parameters
            Lowprio_Scheduler.ch2_sequence_length= seq_length;
            Lowprio_Scheduler.ch2_toggle_counter= seq_length;


            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH2_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH2_CYCLE_MASK);

            break;

        case LOWPRIO_CH3:

            //save command
            Lowprio_Scheduler.ch3_delay1_us= delay1_us;
            Lowprio_Scheduler.ch3_delay2_us= delay2_us;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH3_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH3_CYCLE_MASK);

            break;

        case LOWPRIO_CH4:

            //save command
            Lowprio_Scheduler.ch4_delay1_us= delay1_us;
            Lowprio_Scheduler.ch4_delay2_us= delay2_us;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH4_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH4_CYCLE_MASK);

            break;

        default:
            break;

    }

    //activate lowprio_scheduler for the first delay
    lowprio_scheduler_set_channel(target_ch, callback_funct, TOGGLE, delay1_us);
}


/**
activates toggle mode on the commanded lowprio_scheduler channel (free running forever)

the lowprio_scheduler will automatically reload with alternating delay1/delay_2 and request toggling the desired pin

invoke lowprio_scheduler_reset_channel() to end toggle mode

the callback function must support "TOGGLE" as parameter
*/
void lowprio_scheduler_togglemode_channel(lowprio_scheduler_channel_t target_ch, void (* callback_funct)(output_pin_t), U32 delay1_us, U32 delay2_us)
{
    switch(target_ch)
    {
        case LOWPRIO_CH1:

            //save command
            Lowprio_Scheduler.ch1_delay1_us= delay1_us;
            Lowprio_Scheduler.ch1_delay2_us= delay2_us;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH1_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH1_CYCLE_MASK);

            break;

        case LOWPRIO_CH2:

            //save command
            Lowprio_Scheduler.ch2_delay1_us= delay1_us;
            Lowprio_Scheduler.ch2_delay2_us= delay2_us;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH2_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH2_CYCLE_MASK);

            break;

        case LOWPRIO_CH3:

            //save command
            Lowprio_Scheduler.ch3_delay1_us= delay1_us;
            Lowprio_Scheduler.ch3_delay2_us= delay2_us;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH3_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH3_CYCLE_MASK);

            break;

        case LOWPRIO_CH4:

            //save command
            Lowprio_Scheduler.ch4_delay1_us= delay1_us;
            Lowprio_Scheduler.ch4_delay2_us= delay2_us;

            //register toggle mode, cycle begin
            Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH4_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH4_CYCLE_MASK);

            break;

        default:
            break;

    }

    //activate lowprio_scheduler for the first delay
    lowprio_scheduler_set_channel(target_ch, callback_funct, TOGGLE, delay1_us);
}



void lowprio_scheduler_set_channel(lowprio_scheduler_channel_t target_ch, void (* callback_funct)(output_pin_t), output_pin_t action, U32 delay_us)
{

    volatile U64 compare;
    VU32 now;
    volatile U64 remain;

    //get current timer value -> crucial part
    now= TIM11->CNT;

    //safety check - clip delay
    if(delay_us > LOWPRIO_SCHEDULER_MAX_PERIOD_US)
    {
        delay_us= LOWPRIO_SCHEDULER_MAX_PERIOD_US;

        //TODO log a warning
    }

    //compare value at delay end in ticks
    compare= now  + (delay_us / LOWPRIO_SCHEDULER_PERIOD_US);

    switch(target_ch)
    {
        case LOWPRIO_CH1:

            TIM11->DIER &= (U16) ~TIM_DIER_CC1IE;

            //store the desired action at delay end
            Lowprio_Scheduler.ch1_callback= callback_funct;
            Lowprio_Scheduler.ch1_action= action;

            if(compare >= 0xFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */

                //amount of ticks after update event
                remain= compare - 0xFFFF;


                /**
                we can not set the new compare value right now, if we will see it in this timer cycle
                */
                if(remain < now)
                {
                    //compare already behind, hits after update
                    //preload function not needed
                    TIM11->CCMR1 &= ~TIM_CCMR1_OC1PE;
                }
                else
                {
                    //set new compare after update event, use a default value by now
                    TIM11->CCR1= (U16) 0x00;

                    //preload function will load new compare value after update event
                    TIM11->CCMR1 |= TIM_CCMR1_OC1PE;
                }

                //set compare register
                TIM11->CCR1= (U16) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM11->CCMR1 &= ~TIM_CCMR1_OC1PE;
                TIM11->CCR1  = (U16) compare;

            }

            //clear pending flags and enable irq
            TIM11->SR    = (U16) ~TIM_SR_CC1IF;
            TIM11->DIER |= (U16) TIM_DIER_CC1IE;

            break;


        case LOWPRIO_CH2:

            TIM11->DIER &= (U16) ~TIM_DIER_CC2IE;

            Lowprio_Scheduler.ch2_callback= callback_funct;
            Lowprio_Scheduler.ch2_action= action;

            if(compare >= 0xFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */
                remain= compare - 0xFFFF;

                if(remain < now)
                {
                    //compare already behind, hits after update
                    TIM11->CCMR1 &= ~TIM_CCMR1_OC2PE;
                }
                else
                {
                    //set new compare after update event
                    TIM11->CCMR1 |= TIM_CCMR1_OC2PE;
                }

                TIM11->CCR2  = (U16) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM11->CCMR1 &= ~TIM_CCMR1_OC2PE;
                TIM11->CCR2  = (U16) compare;

            }

            //clear pending flags and enable irq
            TIM11->SR    = (U16) ~TIM_SR_CC2IF;
            TIM11->DIER |= (U16) TIM_DIER_CC2IE;
            break;


        case LOWPRIO_CH3:

            TIM11->DIER &= (U16) ~TIM_DIER_CC3IE;

            Lowprio_Scheduler.ch3_callback= callback_funct;
            Lowprio_Scheduler.ch3_action= action;

            if(compare >= 0xFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */
                remain= compare - 0xFFFF;

                if(remain < now)
                {
                    //compare already behind, hits after update
                    TIM11->CCMR2 &= ~TIM_CCMR2_OC3PE;
                }
                else
                {
                    //set new compare after update event
                    TIM11->CCMR2 |= TIM_CCMR2_OC3PE;
                }

                TIM11->CCR3  = (U16) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM11->CCMR2 &= ~TIM_CCMR2_OC3PE;
                TIM11->CCR3  = (U16) compare;

            }

            //clear pending flags and enable irq
            TIM11->SR    = (U16) ~TIM_SR_CC3IF;
            TIM11->DIER |= (U16) TIM_DIER_CC3IE;
            break;


        case LOWPRIO_CH4:

            TIM11->DIER &= (U16) ~TIM_DIER_CC4IE;

            Lowprio_Scheduler.ch4_callback= callback_funct;
            Lowprio_Scheduler.ch4_action= action;

            if(compare >= 0xFFFF)
            {
                /**
                the desired timeout will occur in the next timer cycle
                */
                remain= compare - 0xFFFF;

                if(remain < now)
                {
                    //compare already behind, hits after update
                    TIM11->CCMR2 &= ~TIM_CCMR2_OC4PE;
                }
                else
                {
                    //set new compare after update event
                    TIM11->CCMR2 |= TIM_CCMR2_OC4PE;
                }

                TIM11->CCR4  = (U16) remain;

            }
            else
            {
                /**
                the desired timeout will occur in the current timer cycle
                */
                TIM11->CCMR2 &= ~TIM_CCMR2_OC4PE;
                TIM11->CCR4  = (U16) compare;

            }

            //clear pending flags and enable irq
            TIM11->SR    = (U16) ~TIM_SR_CC4IF;
            TIM11->DIER |= (U16) TIM_DIER_CC4IE;
            break;

    default:
        break;

    }

}



void lowprio_scheduler_reset_channel(lowprio_scheduler_channel_t target_ch)
{
    switch(target_ch)
    {
        case LOWPRIO_CH1:

            TIM11->DIER &= (U16) ~TIM_DIER_CC1IE;
            TIM11->SR    = (U16) ~TIM_SR_CC1IF;
            Lowprio_Scheduler.ch1_callback= safe_callback;
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH1_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH1_SEQUENCE_MASK);
            break;

        case LOWPRIO_CH2:

            TIM11->DIER &= (U16) ~TIM_DIER_CC2IE;
            TIM11->SR    = (U16) ~TIM_SR_CC2IF;
            Lowprio_Scheduler.ch2_callback= safe_callback;
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH2_ENABLE_MASK);
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH2_SEQUENCE_MASK);
            break;

        case LOWPRIO_CH3:

            TIM11->DIER &= (U16) ~TIM_DIER_CC3IE;
            TIM11->SR    = (U16) ~TIM_SR_CC3IF;
            Lowprio_Scheduler.ch3_callback= safe_callback;
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH3_ENABLE_MASK);
            break;

        case LOWPRIO_CH4:

            TIM11->DIER &= (U16) ~TIM_DIER_CC4IE;
            TIM11->SR    = (U16) ~TIM_SR_CC4IF;
            Lowprio_Scheduler.ch4_callback= safe_callback;
            Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH4_ENABLE_MASK);
            break;

        default:
            break;

    }
}





void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    VU32 toggle_delay;

    if( TIM11->SR & TIM_SR_CC1IF)
    {
        //clear irq pending bit
        TIM11->SR= (U16) ~TIM_SR_CC1IF;

        /**
        channel 1
        */
        Lowprio_Scheduler.ch1_callback(Lowprio_Scheduler.ch1_action);

        //disable compare irq
        TIM11->DIER &= (U16) ~TIM_DIER_CC1IE;

        //re enable channel in toggle mode
        if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH1_ENABLE_MASK))
        {
            /**
            check which delay to enable
            generally this depends on the state of the CYCLE bit, only in SEQ mode we select delay3 when the toggle_counter has expired
            */
            //check if we are in SEQ mode
            if( (Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH1_SEQUENCE_MASK)) && (Lowprio_Scheduler.ch1_toggle_counter == 0) )
            {
                //select "pause" interval
                toggle_delay= Lowprio_Scheduler.ch1_delay3_us;

                //continue with delay1 after delay3
                Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH1_CYCLE_MASK);

                //reset counter
                Lowprio_Scheduler.ch1_toggle_counter= Lowprio_Scheduler.ch1_sequence_length;
            }
            //check the CYCLE bit
            else if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH1_CYCLE_MASK))
            {
                //CYCLE bit is set, we are at the end of the second delay
                toggle_delay= Lowprio_Scheduler.ch1_delay1_us;
                Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH1_CYCLE_MASK);

                //count the repetition, if in SEQ mode
                if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH1_SEQUENCE_MASK))
                {
                    Lowprio_Scheduler.ch1_toggle_counter--;
                }

            }
            else
            {
                //CYCLE bit is reset, we are at the end of the first delay
                toggle_delay= Lowprio_Scheduler.ch1_delay2_us;
                Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH1_CYCLE_MASK);
            }

            //re enable
            lowprio_scheduler_set_channel(LOWPRIO_CH1, Lowprio_Scheduler.ch1_callback, TOGGLE, toggle_delay);
        }
        else
        {
            //single shot mode end -> set a safe callback function
            Lowprio_Scheduler.ch1_callback= safe_callback;
        }
    }


    if( TIM11->SR & TIM_SR_CC2IF)
    {
        /**
        channel 2
        */
        Lowprio_Scheduler.ch2_callback(Lowprio_Scheduler.ch2_action);

        //clear irq pending bit
        TIM11->SR= (U16) ~TIM_SR_CC2IF;

        //disable compare irq
        TIM11->DIER &= (U16) ~TIM_DIER_CC2IE;

        //re enable channel in toggle mode
        if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH2_ENABLE_MASK))
        {
            /**
            check which delay to enable
            generally this depends on the state of the CYCLE bit, only in SEQ mode we select delay3 when the toggle_counter has expired
            */
            //check if we are in SEQ mode
            if( (Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH2_SEQUENCE_MASK)) && (Lowprio_Scheduler.ch2_toggle_counter == 0) )
            {
                //select "pause" interval
                toggle_delay= Lowprio_Scheduler.ch2_delay3_us;

                //continue with delay1 after delay3
                Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH2_CYCLE_MASK);

                //reset counter
                Lowprio_Scheduler.ch2_toggle_counter= Lowprio_Scheduler.ch2_sequence_length;
            }
            //check the CYCLE bit
            else if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH2_CYCLE_MASK))
            {
                //CYCLE bit is set, we are at the end of the second delay
                toggle_delay= Lowprio_Scheduler.ch2_delay1_us;
                Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH2_CYCLE_MASK);

                //count the repetition, if in SEQ mode
                if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH2_SEQUENCE_MASK))
                {
                    Lowprio_Scheduler.ch2_toggle_counter--;
                }

            }
            else
            {
                //CYCLE bit is reset, we are at the end of the first delay
                toggle_delay= Lowprio_Scheduler.ch2_delay2_us;
                Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH2_CYCLE_MASK);
            }

            //re enable
            lowprio_scheduler_set_channel(LOWPRIO_CH2, Lowprio_Scheduler.ch2_callback, TOGGLE, toggle_delay);
        }
        else
        {
            //single shot mode end -> set a safe callback function
            Lowprio_Scheduler.ch2_callback= safe_callback;
        }
    }

    if( TIM11->SR & TIM_SR_CC3IF)
    {
        /**
        channel 3
        */
        Lowprio_Scheduler.ch3_callback(Lowprio_Scheduler.ch3_action);

        //clear irq pending bit
        TIM11->SR= (U16) ~TIM_SR_CC3IF;

        //disable compare irq
        TIM11->DIER &= (U16) ~TIM_DIER_CC3IE;

        //re enable channel in toggle mode
        if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH3_ENABLE_MASK))
        {
            //check whether the first or second delay to enable
            if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH3_CYCLE_MASK))
            {
                //CYCLE bit is set, we are at the end of the second delay
                toggle_delay= Lowprio_Scheduler.ch3_delay1_us;
                Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH3_CYCLE_MASK);
            }
            else
            {
                //CYCLE bit is reset, we are at the end of the first delay
                toggle_delay= Lowprio_Scheduler.ch3_delay2_us;
                Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH3_CYCLE_MASK);
            }

            //re enable
            lowprio_scheduler_set_channel(LOWPRIO_CH3, Lowprio_Scheduler.ch3_callback, TOGGLE, toggle_delay);
        }
        else
        {
            //set a safe callback function
            Lowprio_Scheduler.ch3_callback= safe_callback;
        }
    }

    if( TIM11->SR & TIM_SR_CC4IF)
    {
        /**
        channel 4
        */
        Lowprio_Scheduler.ch4_callback(Lowprio_Scheduler.ch4_action);

        //clear irq pending bit
        TIM11->SR= (U16) ~TIM_SR_CC4IF;

        //disable compare irq
        TIM11->DIER &= (U16) ~TIM_DIER_CC4IE;

        //re enable channel in toggle mode
        if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH4_ENABLE_MASK))
        {
            //check whether the first or second delay to enable
            if(Lowprio_Scheduler.toggle_ctrl & (1 << TOGGLE_CH4_CYCLE_MASK))
            {
                //CYCLE bit is set, we are at the end of the second delay
                toggle_delay= Lowprio_Scheduler.ch4_delay1_us;
                Lowprio_Scheduler.toggle_ctrl &= ~(1 << TOGGLE_CH4_CYCLE_MASK);
            }
            else
            {
                //CYCLE bit is reset, we are at the end of the first delay
                toggle_delay= Lowprio_Scheduler.ch4_delay2_us;
                Lowprio_Scheduler.toggle_ctrl |= (1 << TOGGLE_CH4_CYCLE_MASK);
            }

            //re enable
            lowprio_scheduler_set_channel(LOWPRIO_CH4, Lowprio_Scheduler.ch4_callback, TOGGLE, toggle_delay);
        }
        else
        {
            //set a safe callback function
            Lowprio_Scheduler.ch4_callback= safe_callback;
        }
    }

}
