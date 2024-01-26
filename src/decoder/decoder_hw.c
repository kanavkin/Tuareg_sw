#include <Tuareg_platform.h>
#include <Tuareg.h>

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
-   if a timer overflow occurs, the engine most certainly has stalled / is not running

*/


/**
use 16 bit TIM9 for crank pickup signal decoding
the timer is started as a crank pickup signal edge is detected
the crank noise filter masks the crank pickup irq until the compare event is triggered
the required timer compare value is part of the config and must be provided as a parameter
*/
void decoder_start_timer()
{
    // clear flags
    TIM9->SR= (U16) 0;

    //reset counter
    TIM9->CNT= (U16) 0;

    //set prescaler
    decoder_set_timer_prescaler(DECODER_TIMER_PRESCALER, DECODER_TIMER_PERIOD_US, DECODER_TIMER_OVERFLOW_MS);

    //enable output compare for exti
    TIM9->CCR1= (U16) Decoder_Setup.crank_noise_filter;

    //switch decoder mode to discontinuous
    decoder_set_timer_continuous_mode_off();

    Decoder_hw.current_timer_value= 0;
    Decoder_hw.prev1_timer_value= 0;
    Decoder_hw.prev2_timer_value= 0;
    Decoder_hw.captured_positions_cont= 0;


    //enable overflow interrupt
    TIM9->DIER |= TIM_DIER_UIE;

    //enable compare 1 event
    TIM9->DIER |= TIM_DIER_CC1IE;

    //start timer counter
    TIM9->CR1 |= TIM_CR1_CEN;
}


void decoder_set_timer_prescaler(U32 Prescaler, U32 Period_us, U32 Overflow_ms)
{
    TIM9->PSC= (U16) (Prescaler -1);

    //store prescaler value with respect to 16 Bit hw register
    Decoder_hw.timer_prescaler= Prescaler & 0xFFFF;

    Decoder_hw.timer_period_us= Period_us;
    Decoder_hw.timer_overflow_ms= Overflow_ms;
}


void decoder_stop_timer()
{
    TIM9->CR1 &= ~TIM_CR1_CEN;

    //disable timer interrupts
    TIM9->DIER= (U16) 0;

    //delete interrupt flags
    TIM9->SR = (U16) 0;
}


void update_crank_noisefilter(U32 timer_base)
{
    U32 compare;

    //calculate the compare value
    compare= timer_base + Decoder_Setup.crank_noise_filter;

    //check if this value can be reached in this timer cycle
    VitalAssert( compare < 0xFFFF, TID_DECODER_HW, DECODER_LOC_HW_UPD_CRANK_NOISEF_ARG);

    //disable compare 1 event
    TIM9->DIER &= ~TIM_DIER_CC1IE;

    //enable output compare for exti
    TIM9->CCR1= (U16) compare;

    //clear the pending flag
    TIM9->SR = (U16) ~TIM_FLAG_CC1;

    //enable compare 1 event
    TIM9->DIER |= TIM_DIER_CC1IE;
}


void update_cam_noisefilter(U32 timer_base)
{
    U32 compare;

    //calculate the compare value
    compare= timer_base + Decoder_Setup.cam_noise_filter;

    //check if this value can be reached in this timer cycle
    VitalAssert( compare < 0xFFFF, TID_DECODER_HW, DECODER_LOC_HW_UPD_CAM_NOISEF_ARG);

    //disable compare 2 event
    TIM9->DIER &= ~TIM_DIER_CC2IE;

    //enable output compare for exti
    TIM9->CCR2= (U16) compare;

    //clear the pending flag
    TIM9->SR = (U16) ~TIM_FLAG_CC2;

    //enable compare 1 event
    TIM9->DIER |= TIM_DIER_CC2IE;
}



/**
crank pickup irq helper functions
(hardware dependent)
*/
void decoder_mask_crank_irq()
{
    //Bit in IMR reset -> masked (disabled!)
    EXTI->IMR &= ~EXTI_IMR_MR0;
}

void decoder_unmask_crank_irq()
{
    //clear the pending flag
    EXTI->PR= EXTI_Line0;

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
    Decoder_hw.crank_pickup_sensing= SENSING_RISE;
}

void set_crank_pickup_sensing_fall()
{
    EXTI->FTSR |= EXTI_FTSR_TR0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    Decoder_hw.crank_pickup_sensing= SENSING_FALL;
}

void set_crank_pickup_sensing_disabled()
{
    EXTI->RTSR &= ~EXTI_RTSR_TR0;
    EXTI->FTSR &= ~EXTI_FTSR_TR0;
    Decoder_hw.crank_pickup_sensing= SENSING_DISABLED;
}

/**
cis sensing helper functions
(the hardware dependent part)
*/
void set_cis_sensing_rise()
{
    EXTI->RTSR |= EXTI_RTSR_TR1;
    EXTI->FTSR &= ~EXTI_FTSR_TR1;
    Decoder_hw.cis_sensing= SENSING_RISE;
}

void set_cis_sensing_fall()
{
    EXTI->FTSR |= EXTI_FTSR_TR1;
    EXTI->RTSR &= ~EXTI_RTSR_TR1;
    Decoder_hw.cis_sensing= SENSING_FALL;
}

void set_cis_sensing_disabled()
{
    EXTI->RTSR &= ~EXTI_RTSR_TR1;
    EXTI->FTSR &= ~EXTI_FTSR_TR1;
    Decoder_hw.cis_sensing= SENSING_DISABLED;
}


/**
select the signal edge that will trigger the decoder (pickup sensing)
masks the crank pickup irq!
*/
void decoder_set_crank_pickup_sensing(decoder_sensing_t sensing)
{
    //disable irq for setup
    decoder_mask_crank_irq();

    switch(sensing)
    {
    case SENSING_RISE:
                set_crank_pickup_sensing_rise();
                break;

    case SENSING_FALL:
                set_crank_pickup_sensing_fall();
                break;

    case SENSING_INVERT:
                if(Decoder_hw.crank_pickup_sensing == SENSING_RISE)
                {
                    set_crank_pickup_sensing_fall();
                }
                else if(Decoder_hw.crank_pickup_sensing == SENSING_FALL)
                {
                    set_crank_pickup_sensing_rise();
                }
                else
                {
                    //invalid usage
                    set_crank_pickup_sensing_disabled();
                    Fatal(TID_DECODER_HW, DECODER_LOC_HW_SET_CRANK_SENSING_INVERT);
                }

                break;

    default:
                //invalid usage
                set_crank_pickup_sensing_disabled();
                Fatal(TID_DECODER_HW, DECODER_LOC_HW_SET_CRANK_SENSING_ARG);
                break;
    }

}

/**
select the signal edge that will trigger the cylinder identification sensor
masks the cis irq!
*/
void decoder_set_cis_sensing(decoder_sensing_t sensing)
{
    //disable irq for setup
    decoder_mask_cis_irq();

    switch(sensing)
    {
    case SENSING_RISE:
                set_cis_sensing_rise();
                break;

    case SENSING_FALL:
                set_cis_sensing_fall();
                break;

    case SENSING_INVERT:
                if(Decoder_hw.cis_sensing == SENSING_RISE)
                {
                    set_cis_sensing_fall();
                }
                else if(Decoder_hw.cis_sensing == SENSING_FALL)
                {
                    set_cis_sensing_rise();
                }
                else
                {
                    //invalid usage
                    set_cis_sensing_disabled();
                    Fatal(TID_DECODER_HW, DECODER_LOC_HW_SET_CAM_SENSING_INVERT);
                }

                break;

    default:
                //invalid usage
                set_cis_sensing_disabled();
                Fatal(TID_DECODER_HW, DECODER_LOC_HW_SET_CAM_SENSING_ARG);
                break;
    }

}


void trigger_decoder_irq()
{
    /**
    diagnostics
    */
    decoder_diag_log_event(DDIAG_UPDATE_IRQ_CALLS);

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
    //clock tree setup
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_EXTIEN | RCC_APB2ENR_SYSCFGEN| RCC_APB2ENR_TIM9EN;

    //set input mode for crank pickup and cylinder identification sensor
    GPIO_configure(GPIOB, 0, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_MID, GPIO_PULL_UP);
    GPIO_configure(GPIOB, 1, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_DOWN);

    //map GPIOB0 to EXTI line 0 (crank) and GPIOB1 to EXTI line 1 (cam)
    SYSCFG_map_EXTI(0, EXTI_MAP_GPIOB);
    SYSCFG_map_EXTI(1, EXTI_MAP_GPIOB);

    //configure EXTI polarity, but keep irqs masked for now
    decoder_set_crank_pickup_sensing(Decoder_Setup.key_begin_sensing);
    decoder_set_cis_sensing(Decoder_Setup.lobe_begin_sensing);

    //reset timer values until TDC has been detected
    Decoder_hw.state.timer_continuous_mode= false;
    Decoder_hw.current_timer_value= 0;
    Decoder_hw.prev1_timer_value= 0;
    Decoder_hw.prev2_timer_value= 0;
    Decoder_hw.captured_positions_cont= 0;

    //enable sw irq on exti line 2
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


//returns the current decoder timestamp
U32 decoder_get_timestamp()
{
    return TIM9->CNT;
}

void decoder_reset_timestamp()
{
    TIM9->CNT= (U16) 0;
}


void decoder_set_timer_continuous_mode_on()
{
    Decoder_hw.state.timer_continuous_mode= true;

    Decoder_hw.current_timer_value= 0;
    Decoder_hw.prev1_timer_value= 0;
    Decoder_hw.prev2_timer_value= 0;
    Decoder_hw.captured_positions_cont= 0;
}


void decoder_set_timer_continuous_mode_off()
{
    Decoder_hw.state.timer_continuous_mode= false;

    Decoder_hw.current_timer_value= 0;
    Decoder_hw.prev1_timer_value= 0;
    Decoder_hw.prev2_timer_value= 0;
    Decoder_hw.captured_positions_cont= 0;
}


void decoder_request_timer_reset()
{
    Decoder_hw.state.timer_reset_req= true;
}



void disable_decoder_hw()
{
    //disable crank pickup irq
    NVIC_DisableIRQ(EXTI0_IRQn);
    decoder_mask_crank_irq();
    set_crank_pickup_sensing_disabled();

    //disable cis irq
    NVIC_DisableIRQ(EXTI1_IRQn);
    decoder_mask_cis_irq();
    set_cis_sensing_disabled();

    //disable timer 9 compare 1 irq
    NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
    decoder_stop_timer();

    //disable sw exti irq
    NVIC_DisableIRQ(EXTI2_IRQn);
}





/******************************************************************************************************************************
crankshaft position sensor
 ******************************************************************************************************************************/
void EXTI0_IRQHandler(void)
{
    VU32 timer_buffer;

//    __disable_irq();

    //save timer value
    timer_buffer= decoder_get_timestamp();

    //clear the pending flag after saving timer value to minimize measurement delay
    EXTI->PR= EXTI_Line0;

    //reset decoder timer only if commanded
    if((Decoder_hw.state.timer_continuous_mode == false) || (Decoder_hw.state.timer_reset_req == true))
    {
        //reset timer
        decoder_reset_timestamp();

        Decoder_hw.captured_positions_cont= 1;

        //use the known timer value as the source for noise filter calculation to prevent race conditions
        update_crank_noisefilter(0);

        //request has been processed
        Decoder_hw.state.timer_reset_req= false;
    }
    else
    {
        //counter continues counting
        Decoder_hw.captured_positions_cont += 1;

        //use the known timer value as the source for noise filter calculation to prevent race conditions
        update_crank_noisefilter(timer_buffer);
    }

    //update the continuous timer value
    Decoder_hw.prev2_timer_value= Decoder_hw.prev1_timer_value;
    Decoder_hw.prev1_timer_value= Decoder_hw.current_timer_value;
    Decoder_hw.current_timer_value= timer_buffer;


    //diagnostics
    decoder_diag_log_event(DDIAG_CRK_EXTI_EVENTS);

 //   __enable_irq();


    //call the logic handler
    decoder_crank_handler();

}


/******************************************************************************************************************************
Timer 9 - decoder control:
    -timer 9 compare event 1 --> enable external interrupt for pickup sensor
    -timer 9 compare event 2 --> enable external interrupt for cis
    -timer 9 update event --> overflow interrupt occurs when no signal from crankshaft pickup has been received for more then 4s
 ******************************************************************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{
    //TIM9 compare event
    if((TIM9->DIER & TIM_DIER_CC1IE) && (TIM9->SR & TIM_FLAG_CC1))
    {
        //clear the pending flag
        TIM9->SR = (U16) ~TIM_FLAG_CC1;

        //diagnostics
        decoder_diag_log_event(DDIAG_CRK_NOISEF_EVENTS);

        //call the logic handler
        decoder_crank_noisefilter_handler();
    }


    //TIM9 compare event
    if((TIM9->DIER & TIM_DIER_CC2IE) && (TIM9->SR & TIM_FLAG_CC2))
    {
        //clear the pending flag
        TIM9->SR = (U16) ~TIM_FLAG_CC2;

        //diagnostics
        decoder_diag_log_event(DDIAG_CAM_NOISEF_EVENTS);

        //call the logic handler
        decoder_cis_noisefilter_handler();
    }


    //TIM9 update event
    if((TIM9->DIER & TIM_DIER_UIE) && (TIM9->SR & TIM_FLAG_Update))
    {
        //clear the pending flag
        TIM9->SR = (U16) ~TIM_FLAG_Update;

        //timing destroyed
        Decoder_hw.prev2_timer_value= 0;
        Decoder_hw.prev1_timer_value= 0;
        Decoder_hw.current_timer_value= 0;
        Decoder_hw.captured_positions_cont= 0;

        //diagnostics
        decoder_diag_log_event(DDIAG_TIM_UPDATE_EVENTS);

        //call the logic handler
        decoder_crank_timeout_handler();

    }

}


/******************************************************************************************************************************
cylinder identification sensor
 ******************************************************************************************************************************/
void EXTI1_IRQHandler(void)
{
    //clear the pending flag
    EXTI->PR= EXTI_Line1;

    //diagnostics
    decoder_diag_log_event(DDIAG_CAM_EXTI_EVENTS);

    //call the logic handler
    decoder_cis_handler();
}



