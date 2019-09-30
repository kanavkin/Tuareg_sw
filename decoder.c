#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "decoder.h"
#include "uart.h"
#include "Tuareg.h"

#include "debug.h"

volatile decoder_t Decoder;

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
void start_decoder_timer()
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


void stop_decoder_timer()
{
    TIM9->CR1 &= ~TIM_CR1_CEN;
}



void mask_crank_pickup_irq()
{
    EXTI->IMR &= ~EXTI_IMR_MR0;
}

void unmask_crank_pickup_irq()
{
    EXTI->IMR |= EXTI_IMR_MR0;
}

void mask_cis_irq()
{
    EXTI->IMR &= ~EXTI_IMR_MR1;
}

void unmask_cis_irq()
{
    EXTI->IMR |= EXTI_IMR_MR1;
}



/**
pickup sensing at GPIOB-0
*/
void set_crank_pickup_sensing(sensing_t sensing)
{
    //disable irq for setup
    mask_crank_pickup_irq();

    switch(sensing)
    {
    case RISE:
                EXTI->RTSR |= EXTI_RTSR_TR0;
                EXTI->FTSR &= ~EXTI_FTSR_TR0;
                Decoder.crank_pickup_sensing= RISE;
                break;

    case FALL:
                EXTI->FTSR |= EXTI_FTSR_TR0;
                EXTI->RTSR &= ~EXTI_RTSR_TR0;
                Decoder.crank_pickup_sensing= FALL;
                break;

    case INVERT:
                if(Decoder.crank_pickup_sensing == RISE)
                {
                    //switch to fall
                    EXTI->FTSR |= EXTI_FTSR_TR0;
                    EXTI->RTSR &= ~EXTI_RTSR_TR0;
                    Decoder.crank_pickup_sensing= FALL;
                }
                else
                {
                    //to rise
                    EXTI->RTSR |= EXTI_RTSR_TR0;
                    EXTI->FTSR &= ~EXTI_FTSR_TR0;
                    Decoder.crank_pickup_sensing= RISE;
                }

                break;

    default:
                //disabled
                EXTI->RTSR &= ~EXTI_RTSR_TR0;
                EXTI->FTSR &= ~EXTI_FTSR_TR0;
                Decoder.crank_pickup_sensing= DISABLED;
                break;
    }

}



/**
evaluate sync_buffer_key / sync_buffer_gap ratio
to get trigger wheel sync
*/
U32 check_for_key_a()
{
    U32 sync_ratio;

    if(Decoder.sync_buffer_gap == 0)
    {
        return (U32) 0;
    }

    /*
    fixed point maths -> ratio in percent
    sync_buffer_xx is within a 1..65535 range, U32 is wide enough
    */
    sync_ratio= (Decoder.sync_buffer_key * 100) / Decoder.sync_buffer_gap;

    if( (sync_ratio >= SYNC_RATIO_MIN) && (sync_ratio <= SYNC_RATIO_MAX) )
    {
        //its key A!
        return 0xFFFFFFFF;
    }
    else
    {
        return (U32) 0;
    }
}


/**
    using
    - GPIOB0 for pickup sensing
    -> EXTI0_IRQ
    enables pickup sensor interrupt
*/
volatile decoder_t * init_decoder()
{
    //interface variables
    Decoder.sync_mode= INIT;
    Decoder.crank_position= UNDEFINED_POSITION;
    Decoder.phase= PHASE_UNDEFINED;

    Decoder.diag_positions_cis_phased =0;
    Decoder.diag_positions_cis_undefined =0;
    Decoder.diag_positions_crank_async =0;
    Decoder.diag_positions_crank_synced =0;

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
    set_crank_pickup_sensing(SENSING_KEY_BEGIN);
    unmask_crank_pickup_irq();

    //configure cylinder identification sensor EXTI (only rising edge), but keep c.i.s. irq masked for now
    EXTI->RTSR |= EXTI_RTSR_TR1;
    EXTI->FTSR &= ~EXTI_FTSR_TR1;
    mask_cis_irq();

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

    return &Decoder;
}



/******************************************************************************************************************************
crankshaft position sensor
 ******************************************************************************************************************************/
void EXTI0_IRQHandler(void)
{
    VU32 timer_buffer;

    //save timer value and start over
    timer_buffer= TIM9->CNT;
    TIM9->CNT= (U16) 0;

    //clear the pending flag after saving timer value to minimize measurement delay
    EXTI->PR= EXTI_Line0;

    //DEBUG
    set_debug_led(ON);

    switch(Decoder.sync_mode) {

        case INIT:

            // this is the first impulse ever captured -> no timing info available
            start_decoder_timer();
            set_crank_pickup_sensing(SENSING_KEY_END);
            Decoder.sync_mode= ASYNC_KEY;
            Decoder.crank_position= UNDEFINED_POSITION;
            Decoder.phase= PHASE_UNDEFINED;
            Decoder.cycle_timing_buffer= 0UL;
            Decoder.cycle_timing_counter= 0UL;
            break;


        case ASYNC_KEY:

            /* we are at the end of a key -> key duration captured
            in ASYNC mode we try to find segment A */
            Decoder.sync_buffer_key= timer_buffer;
            set_crank_pickup_sensing(SENSING_KEY_BEGIN);
            Decoder.sync_mode= ASYNC_GAP;
            Decoder.crank_position= UNDEFINED_POSITION;

            //collect diagnostic data
            Decoder.diag_positions_crank_async++;

            break;


        case ASYNC_GAP:

            // we are at the beginning of a key -> timer captured gap duration, next int will be on key end anyways
            Decoder.sync_buffer_gap= timer_buffer;
            set_crank_pickup_sensing(SENSING_KEY_END);

            if( check_for_key_a() )
            {
                // it was key A -> we have SYNC now!
                Decoder.sync_mode= SYNC;
                Decoder.crank_position= POSITION_B1;
                Decoder.phase= PHASE_UNDEFINED;

                //prepare rpm calculation
                Decoder.cycle_timing_counter= 0UL;
                Decoder.cycle_timing_buffer= 0UL;
                Decoder.engine_rpm= 0UL;
            }
            else
            {
                //any other key
                Decoder.sync_mode= ASYNC_KEY;
                Decoder.crank_position= UNDEFINED_POSITION;
            }

            //collect diagnostic data
            Decoder.diag_positions_crank_async++;

            break;


        case SYNC:

            //update crank_position
            if(Decoder.crank_position == POSITION_D2)
            {
                //crank cycle end -> turn around
                Decoder.crank_position= POSITION_A1;
            }
            else
            {
                Decoder.crank_position++;
            }

            // update crank sensing
            set_crank_pickup_sensing(INVERT);

            /*
            collect data for engine rpm calculation
            (in every crank 360° revolution we collect 8 timer values)
            range check: 2^3*2^16=2^19 -> 2^19 < 2^32 -> fits to 32 bit variable
            */
            Decoder.cycle_timing_buffer += timer_buffer;
            Decoder.cycle_timing_counter++;

            //2 full crank revolutions captured?
            if(Decoder.cycle_timing_counter == 16)
            {
                //this is for 720 deg
                Decoder.engine_rpm= (120UL * SystemCoreClock) / (DECODER_TIMER_PSC * Decoder.cycle_timing_buffer);

                //reset calculation
                Decoder.cycle_timing_buffer= 0;
                Decoder.cycle_timing_counter= 0;
            }

            //collect diagnostic data
            Decoder.diag_positions_crank_synced++;


            /**
            per-position decoder housekeeping actions:
            crank_position is the crank position that was reached at the beginning of this interrupt
            */
            switch(Decoder.crank_position) {

            case POSITION_A2:

                // store key length for sync check
                Decoder.sync_buffer_key= timer_buffer;
                break;


            case POSITION_B1:

                //do sync check
                Decoder.sync_buffer_gap= timer_buffer;

                if( !check_for_key_a() )
                {
                    //sync check failed! -> prepare for the next trigger condition
                    Decoder.crank_position= UNDEFINED_POSITION;
                    Decoder.sync_mode= ASYNC_KEY;
                    set_crank_pickup_sensing(SENSING_KEY_BEGIN);

                    //engine phase measurement is upset too
                    Decoder.phase= PHASE_UNDEFINED;
                    mask_cis_irq();

                    //collect diagnostic data
                    Decoder.diag_sync_lost_events++;
                }
                break;

            default:
                //any other position
                break;

            }


            /**
            cylinder identification sensor handling
            running only in sync with crank, if we lose sync in POSITION_B1 sync check:
            -> the irq enable/disable branches will not be executed as crank_position defaults to UNDEFINED
            -> the c.i.s. irq is masked already
            */
            if(Decoder.crank_position == CYLINDER_SENSOR_ENA_POSITION)
            {
                //clear the pending flag
                EXTI->PR= EXTI_Line1;

                //enable c.i.s irq
                unmask_cis_irq();
            }
            else if(Decoder.crank_position == CYLINDER_SENSOR_DISA_POSITION)
            {
                //disable c.i.s irq
                mask_cis_irq();

                //no c.i.s trigger event has been detected -> cylinder #2 working or broken sensor
                if(Decoder.phase == CYL1_WORK)
                {
                    //sensor gives valid data
                    Decoder.phase= CYL2_WORK;
                }
                else if(Decoder.phase == CYL2_WORK)
                {
                    //no alternating signal -> sensor error
                    Decoder.phase= PHASE_UNDEFINED;

                    //collect diagnostic data
                    Decoder.diag_phase_lost_events++;
                }
            }

            //collect diagnostic data
            if(Decoder.phase == PHASE_UNDEFINED)
            {
                Decoder.diag_positions_cis_undefined++;
            }
            else
            {
                Decoder.diag_positions_cis_phased++;
            }


            /**
            finally trigger the sw irq 2 for decoder output processing
            (ca. 3.2us after trigger event)
            the irq will be triggered anyways to react on sync failures!
            */
            EXTI->SWIER= EXTI_SWIER_SWIER2;
            break;

        }

        //reset timeout counter, we just saw a trigger condition
        Decoder.timeout_count= 0UL;

        /**
        noise filter
        mask crank pickup irq, timer 2 compare will enable it again
        crank pickup irq is disabled after each set_crank_pickup_sensing() call
        */
        mask_crank_pickup_irq();

}


/******************************************************************************************************************************
Timer 9 - decoder control:
    -timer 9 compare event 1 --> enable external interrupt for pickup sensor
    -timer 9 update event --> overflow interrupt occurs when no signal from crankshaft pickup has been received for more then 4s
 ******************************************************************************************************************************/
void TIM9_IRQHandler(void)
{
    //compare event
    if( TIM9->SR & TIM_IT_CC1)
    {
        TIM9->SR = (U16) ~TIM_IT_CC1;

        //crank noise filter expired -> re enable crank pickup interrupt
        unmask_crank_pickup_irq();
    }



    //update event
    if( TIM9->SR & TIM_IT_Update)
    {
        TIM9->SR = (U16) ~TIM_IT_Update;

        //timer update occurs every 200 ms
        if(Decoder.timeout_count >= DECODER_TIMEOUT)
        {
            //reset decoder
            Decoder.sync_mode= INIT;
            Decoder.crank_position= UNDEFINED_POSITION;

            //reset rpm calculation
            Decoder.cycle_timing_buffer= 0UL;
            Decoder.cycle_timing_counter= 0UL;
            Decoder.engine_rpm= 0UL;

            //trigger sw irq for decoder output processing (ca. 3.2us behind trigger edge)
            EXTI->SWIER= EXTI_SWIER_SWIER2;

            //prepare for the next trigger condition
            stop_decoder_timer();
            set_crank_pickup_sensing(SENSING_KEY_BEGIN);
            unmask_crank_pickup_irq();

            //shut down c.i.s. irq
            mask_cis_irq();
            Decoder.phase= PHASE_UNDEFINED;
        }
        else
        {
            Decoder.timeout_count++;
        }
    }

}


/******************************************************************************************************************************
cylinder identification sensor
 ******************************************************************************************************************************/
void EXTI1_IRQHandler(void)
{
    //clear the pending flag
    EXTI->PR= EXTI_Line1;

    //a trigger condition has been seen on c.i.s -> cylinder #1 working or sensor error
    //first trigger condition allows us to leave UNDEFINED state
    if((Decoder.phase == CYL2_WORK) || (Decoder.phase == PHASE_UNDEFINED))
    {
        //alternating signal -> valid
        Decoder.phase= CYL1_WORK;
    }
    else if(Decoder.phase == CYL1_WORK)
    {
        //invalid signal
        Decoder.phase= PHASE_UNDEFINED;

        //collect diagnostic data
        Decoder.diag_phase_lost_events++;
    }

    //disable c.i.s irq until next turn
    mask_cis_irq();
}



