//#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
//#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
//#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"
#include "Tuareg.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"
#include "decoder_syslog_locations.h"

#include "base_calc.h"

#include "uart.h"
#include "uart_printf.h"
#include "debug_port_messages.h"

#include "decoder_debug.h"



/******************************************************************************************************************************
decoder debug info - internal state
******************************************************************************************************************************/
void decoder_debug_show_internals(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\n*******************************\r\ndecoder interface:");

    print(Port, "\r\ncrank period (us): ");
    printf_U(Port, Decoder.out.crank_period_us, NO_PAD);

    print(Port, "\r\ncrank rpm: ");
    printf_U(Port, Decoder.out.crank_rpm, NO_PAD);

    print(Port, "\r\ncrank acceleration: ");
    printf_F32(Port, Decoder.out.crank_acceleration);

    print(Port, "\r\ncrank position: ");
    printf_crkpos(Port, Decoder.out.crank_position);

    print(Port, "\r\nphase: ");
    printf_phase(Port, Decoder.out.phase);

    print(Port, "\r\nstate: pos_valid phase_valid period_valid rpm_valid accel_valid standstill: ");
    UART_Tx(Port, (Decoder.out.flags.position_valid? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.out.flags.phase_valid? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.out.flags.period_valid? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.out.flags.rpm_valid? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.out.flags.accel_valid? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.out.flags.standstill? '1' :'0'));


    print(Port, "\r\n\r\ndecoder hw:");

    print(Port, "\r\ntimer_prescaler: ");
    printf_U(Port, Decoder_hw.timer_prescaler, NO_PAD);

    print(Port, "\r\ntimer_period_us: ");
    printf_U(Port, Decoder_hw.timer_period_us, NO_PAD);

    print(Port, "\r\ntimer_overflow_ms: ");
    printf_U(Port, Decoder_hw.timer_overflow_ms, NO_PAD);

    print(Port, "\r\ncurrent_timer_value: ");
    printf_U(Port, Decoder_hw.current_timer_value, NO_PAD);

    print(Port, "\r\nprev1_timer_value: ");
    printf_U(Port, Decoder_hw.prev1_timer_value, NO_PAD);

    print(Port, "\r\nprev2_timer_value: ");
    printf_U(Port, Decoder_hw.prev2_timer_value, NO_PAD);

    print(Port, "\r\ncaptured_positions_cont: ");
    printf_U(Port, Decoder_hw.captured_positions_cont, NO_PAD);

    print(Port, "\r\ncrank_pickup_sensing: ");
    printf_decoder_sensing(Port, Decoder_hw.crank_pickup_sensing);

    print(Port, "\r\ncis_sensing: ");
    printf_decoder_sensing(Port, Decoder_hw.cis_sensing);

    print(Port, "\r\nhw state: timer_continuous_mode timer_reset_req: ");
    UART_Tx(Port, (Decoder_hw.state.timer_continuous_mode? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder_hw.state.timer_reset_req? '1' :'0'));

    print(Port, "\r\n\r\ndecoder logic");

    print(Port, "\r\ninternal state: TIMEOUT INIT ASYNC KEY GAP SYNC: ");

    if(Decoder.state == DSTATE_SYNC) print(Port, " *SYNC*");
    else printf_U(Port, Decoder.state, NO_PAD);

    print(Port, "\r\ntimeout_count: ");
    printf_U(Port, Decoder.timeout_count, NO_PAD);

    print(Port, "\r\nlast_crank_rpm: ");
    printf_U(Port, Decoder.last_crank_rpm, NO_PAD);

    print(Port, "\r\nlast_crank_acceleration: ");
    printf_F32(Port, Decoder.last_crank_acceleration);


    print(Port, "\r\n\r\nCIS");

    print(Port, "\r\nlobe_begin_timestamp: ");
    printf_U(Port, Decoder.cis.lobe_begin_timestamp, NO_PAD);

    print(Port, "\r\nlobe_end_timestamp: ");
    printf_U(Port, Decoder.cis.lobe_end_timestamp, NO_PAD);

    print(Port, "\r\nsync_counter: ");
    printf_U(Port, Decoder.cis.sync_counter, NO_PAD);

    print(Port, "\r\ndetected_lobe_ends: ");
    printf_U(Port, Decoder.cis.detected_lobe_ends, NO_PAD);

    print(Port, "\r\npreconditions: period_valid failure preconditions_ok: ");
    UART_Tx(Port, (Decoder.cis.flags.period_valid? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.cis.flags.failure? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.cis.flags.preconditions_ok? '1' :'0'));

    print(Port, "\r\ndetected: lobe_begin lobe_end: ");
    UART_Tx(Port, (Decoder.cis.flags.lobe_begin_detected? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.cis.flags.lobe_end_detected? '1' :'0'));

    print(Port, "\r\noutput: triggered phase_match phase_valid: ");
    UART_Tx(Port, (Decoder.cis.flags.triggered? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.cis.flags.phase_match? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Decoder.cis.flags.phase_valid? '1' :'0'));

    print(Port, "\r\n\r\nDecoder Timer 9");

    print(Port, "\r\nDIER: C1 C2 UPD:");
    UART_Tx(Port, (TIM9->DIER & TIM_DIER_CC1IE? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (TIM9->DIER & TIM_DIER_CC2IE? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (TIM9->DIER & TIM_DIER_UIE? '1' :'0'));

    print(Port, "\r\nSR: CC1 CC2 UPD:");
    UART_Tx(Port, (TIM9->SR & TIM_FLAG_CC1? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (TIM9->SR & TIM_FLAG_CC2? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (TIM9->SR & TIM_FLAG_Update? '1' :'0'));

    print(Port, "\r\ncompare 1 crk: ");
    printf_U(Port, TIM9->CCR1, NO_PAD);

    print(Port, "\r\ncompare 2 cis: ");
    printf_U(Port, TIM9->CCR2, NO_PAD);

    print(Port, "\r\ntimestamp: ");
    printf_U(Port, TIM9->CNT, NO_PAD);


    print(Port, "\r\n\r\nEXTI irq masks: CRK_ena CIS_ena SW_ena");
    UART_Tx(Port, (EXTI->IMR & EXTI_IMR_MR0? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (EXTI->IMR & EXTI_IMR_MR1? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (EXTI->IMR & EXTI_IMR_MR2? '1' :'0'));

    print(Port, "\r\nEXTI irq state: CRK_pnd CIS_pnd SW_pnd ");
    UART_Tx(Port, (EXTI->PR & EXTI_Line0? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (EXTI->PR & EXTI_Line1? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (EXTI->PR & EXTI_Line2? '1' :'0'));


}

/******************************************************************************************************************************
decoder helper functions - debug event messages
******************************************************************************************************************************/
#ifdef DECODER_EVENT_DEBUG
#warning decoder event debugging enabled

decoder_event_debug_flags_t Decoder_Events;


void decoder_process_debug_events()
{
    if(Decoder_Events.all_flags > 0)
    {
        DebugMsg_Warning("Decoder Debug Events (sync_lost got_sync timer_overflow standstill): ");
        UART_Tx(DEBUG_PORT, (Decoder_Events.lost_sync? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder_Events.got_sync? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder_Events.timer_overflow? '1' :'0'));
        UART_Tx(DEBUG_PORT, '-');
        UART_Tx(DEBUG_PORT, (Decoder_Events.standstill? '1' :'0'));

        //reset flags
        Decoder_Events.all_flags= 0;
    }
}

 void register_sync_lost_debug_event()
{
    //set flag
    Decoder_Events.lost_sync= true;
}


 void register_got_sync_debug_event()
{
    //set flag
    Decoder_Events.got_sync= true;
}


 void register_timer_overflow_debug_event()
{
    //set flag
    Decoder_Events.timer_overflow= true;
}


 void register_timeout_debug_event()
{
    //set flag
    Decoder_Events.standstill= true;
}

#endif // DECODER_EVENT_DEBUG



/******************************************************************************************************************************
cylinder identification sensor debugging
******************************************************************************************************************************/
#ifdef DECODER_CIS_DEBUG
#warning decoder cis debug enabled

const U32 cDecoder_cis_debug_size= 100;
VU32 decoder_cis_debug_cnt =0;
volatile decoder_cis_debug_t Decoder_cis_debug[100];
volatile bool decoder_cis_debug_freeze= false;


void decoder_cis_debug_next_cycle()
{
    //protect collected data from overwriting
    if(decoder_cis_debug_freeze == true)
    {
        return;
    }

    if(decoder_cis_debug_cnt < cDecoder_cis_debug_size)
    {
        decoder_cis_debug_cnt++;
    }

    if(decoder_cis_debug_cnt >= cDecoder_cis_debug_size)
    {
        decoder_cis_debug_freeze= true;
        DebugMsg_Warning("decoder cis debug capture ready");
        return;
    }

    //prepare clean debug object
    memclr_boctok((void *)&(Decoder_cis_debug[decoder_cis_debug_cnt]), sizeof(decoder_cis_debug_t));
}


void decoder_update_cis_debug(U32 Lobe_interval_us, U32 Lobe_angle_deg)
{
    //protect collected data from overwriting
    if(decoder_cis_debug_freeze == true)
    {
        return;
    }

    //post fence error
    if(decoder_cis_debug_cnt >= cDecoder_cis_debug_size)
    {
        return;
    }

    //copy data
    Decoder_cis_debug[decoder_cis_debug_cnt].cis= Decoder.cis;
    Decoder_cis_debug[decoder_cis_debug_cnt].timer_period_us= Decoder_hw.timer_period_us;
    Decoder_cis_debug[decoder_cis_debug_cnt].crank_period_us= Decoder.out.crank_period_us;

    Decoder_cis_debug[decoder_cis_debug_cnt].lobe_interval_us= Lobe_interval_us;
    Decoder_cis_debug[decoder_cis_debug_cnt].lobe_angle_deg= Lobe_angle_deg;
}


void print_decoder_cis_debug_set(USART_TypeDef * Port, volatile decoder_cis_debug_t * pTarget)
{
    if(pTarget->cis.flags.preconditions_ok == false)
    {
        print(Port, "\r\nPreconditions NOK: crk_period_valid-cis_failure-lobe_begin-lobe_end: ");
        UART_Tx(TS_PORT, (pTarget->cis.flags.period_valid? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->cis.flags.failure? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->cis.flags.lobe_begin_detected? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->cis.flags.lobe_end_detected? '1' :'0'));
    }
    else
    {
        print(Port, "\r\nPreconditions OK: lobe_begin-lobe_end-num_ends: ");
        UART_Tx(TS_PORT, (pTarget->cis.flags.lobe_begin_detected? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->cis.flags.lobe_end_detected? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        printf_U(TS_PORT, pTarget->cis.detected_lobe_ends, NO_PAD | NO_TRAIL);
    }

    if((pTarget->cis.flags.lobe_begin_detected) && (pTarget->cis.flags.lobe_end_detected) && (pTarget->cis.flags.triggered == false))
    {
        print(Port, "\r\nTrigger NOK: cis_triggered: 0");
    }
    else
    {
        print(Port, "\r\nTrigger OK: cis_triggered: ");
        UART_Tx(TS_PORT, (pTarget->cis.flags.triggered? '1' :'0'));
    }

    print(Port, "\r\nPhase match, Phase valid, Sync counter: ");
    UART_Tx(TS_PORT, (pTarget->cis.flags.phase_match? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTarget->cis.flags.phase_valid? '1' :'0'));
    UART_Tx(TS_PORT, ' ');
    printf_U(Port, pTarget->cis.sync_counter, NO_PAD | NO_TRAIL);

    print(Port, "\r\n Timer period, Crank Period: ");
    printf_U(Port, pTarget->timer_period_us, NO_PAD);
    printf_U(Port, pTarget->crank_period_us, NO_PAD | NO_TRAIL);

    print(Port, "\r\n Lobe Begin, End, Interval, Angle: ");
    printf_U(Port, pTarget->cis.lobe_begin_timestamp, NO_PAD);
    printf_U(Port, pTarget->cis.lobe_end_timestamp, NO_PAD);
    printf_U(Port, pTarget->lobe_interval_us, NO_PAD);
    printf_U(Port, pTarget->lobe_angle_deg, NO_PAD | NO_TRAIL);

}


/**
This function has to be available even without cis debugging enabled
*/
void print_decoder_cis_debug_data(USART_TypeDef * Port)
{
    VU32 entry;

    //save data from overwriting
    decoder_cis_debug_freeze= true;

    print(Port, "\r\n\r\nDecoder CIS debug data:\r\n");

    for(entry=0; entry < decoder_cis_debug_cnt; entry++)
    {
        print_decoder_cis_debug_set(Port, &(Decoder_cis_debug[entry]));
        print(Port, "\r\n");
    }

    //clear data
    decoder_cis_debug_cnt= 0;
    decoder_cis_debug_freeze= false;

}

#endif // DECODER_CIS_DEBUG




/******************************************************************************************************************************
timing debugging
******************************************************************************************************************************/
#ifdef DECODER_TIMING_DEBUG
#warning decoder timing debug enabled

const U32 cDecoder_timing_debug_size= 100;
VU32 decoder_timing_debug_cnt =0;
volatile decoder_timing_debug_t Decoder_timing_debug[100];


void decoder_timing_debug_next_cycle()
{
    if(decoder_timing_debug_cnt < cDecoder_timing_debug_size)
    {
        decoder_timing_debug_cnt++;
    }

    if(decoder_timing_debug_cnt >= cDecoder_timing_debug_size)
    {
        DebugMsg_Warning("decoder timing capture ready");
        return;
    }

    //prepare clean debug object
    memclr_boctok((void *)&(Decoder_timing_debug[decoder_timing_debug_cnt]), sizeof(decoder_output_t));
}


void decoder_update_timing_debug()
{
    //post fence error
    if(decoder_timing_debug_cnt >= cDecoder_timing_debug_size)
    {
        return;
    }

    //decoder hw related data
    Decoder_timing_debug[decoder_timing_debug_cnt].hw_period_us= Decoder_hw.timer_period_us;
    Decoder_timing_debug[decoder_timing_debug_cnt].hw_timer_val= Decoder_hw.current_timer_value;

    //decoder output
    Decoder_timing_debug[decoder_timing_debug_cnt].out= Decoder.out;
}















#endif // DECODER_TIMING_DEBUG

