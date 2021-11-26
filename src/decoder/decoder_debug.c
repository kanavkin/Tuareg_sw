#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"
#include "Tuareg.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"
#include "decoder_debug.h"
#include "decoder_syslog_locations.h"

#include "base_calc.h"

#include "uart.h"
#include "uart_printf.h"
#include "debug_port_messages.h"

#include "diagnostics.h"
#include "highspeed_loggers.h"


/*
#ifdef DECODER_TIMING_DEBUG
#warning decoder timing debug enabled
#endif // DECODER_TIMING_DEBUG
*/


#define DECODER_CIS_DEBUG



/******************************************************************************************************************************
cylinder identification sensor debugging
******************************************************************************************************************************/
#ifdef DECODER_CIS_DEBUG

#warning decoder cis debug enabled

#define DECODER_CIS_DEBUG_LEN 100
VU32 decoder_cis_debug_cnt =0;
volatile decoder_cis_debug_t Decoder_cis_debug[DECODER_CIS_DEBUG_LEN];
volatile decoder_cis_debug_t Decoder_cis_debug_dummy;
volatile bool decoder_cis_debug_freeze= false;


/**
returns a pointer to the next debug data cell to be used
*/
volatile decoder_cis_debug_t * get_decoder_cis_debug_storage()
{
    volatile decoder_cis_debug_t * pCurrent= &Decoder_cis_debug_dummy;

    //protect collected data from overwriting
    if(decoder_cis_debug_freeze == true)
    {
        return pCurrent;
    }


    if(decoder_cis_debug_cnt < DECODER_CIS_DEBUG_LEN -1)
    {
        //before the last cell

        //store address of current cell
        pCurrent= &(Decoder_cis_debug[decoder_cis_debug_cnt]);

        //preselect next cell
        decoder_cis_debug_cnt++;

    }
    else if(decoder_cis_debug_cnt == DECODER_CIS_DEBUG_LEN -1)
    {
        //keep address of current cell
        pCurrent= &(Decoder_cis_debug[decoder_cis_debug_cnt]);

        //last cell occupied
        decoder_cis_debug_freeze= true;
        DebugMsg_Warning("CIS capture ready");
    }
    else
    {
        decoder_cis_debug_cnt=0;
    }

    return pCurrent;
}




void print_decoder_cis_debug_set(USART_TypeDef * Port, volatile decoder_cis_debug_t * pTarget)
{
    if(pTarget->flags.preconditions_ok == false)
    {
        print(Port, "\r\nPreconditions NOK: crk_period_valid-cis_failure-lobe_begin-lobe_end: ");
        UART_Tx(TS_PORT, (pTarget->flags.decoder_period_valid? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->flags.cis_failure? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->flags.lobe_begin_detected? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->flags.lobe_end_detected? '1' :'0'));
    }
    else
    {
        print(Port, "\r\nPreconditions OK: lobe_begin-lobe_end: ");
        UART_Tx(TS_PORT, (pTarget->flags.lobe_begin_detected? '1' :'0'));
        UART_Tx(TS_PORT, '-');
        UART_Tx(TS_PORT, (pTarget->flags.lobe_end_detected? '1' :'0'));
    }

    if((pTarget->flags.lobe_begin_detected) && (pTarget->flags.lobe_end_detected) && (pTarget->flags.cis_triggered == false))
    {
        print(Port, "\r\nTrigger NOK: cis_triggered: 0");
    }
    else
    {
        print(Port, "\r\nTrigger OK: cis_triggered: ");
        UART_Tx(TS_PORT, (pTarget->flags.cis_triggered? '1' :'0'));
    }

    print(Port, "\r\nPhase match, Phase valid, Sync counter: ");
    UART_Tx(TS_PORT, (pTarget->flags.phase_match? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTarget->flags.decoder_output_phase_valid? '1' :'0'));
    UART_Tx(TS_PORT, ' ');
    printf_U(Port, pTarget->decoder_cis_sync_counter, NO_PAD | NO_TRAIL);

    print(Port, "\r\n Timer period, Crank Period: ");
    printf_U(Port, pTarget->decoder_timer_period_us, NO_PAD);
    printf_U(Port, pTarget->decoder_crank_period_us, NO_PAD | NO_TRAIL);

    print(Port, "\r\n Lobe Begin, End, Interval, Angle: ");
    printf_U(Port, pTarget->lobe_begin_ts, NO_PAD);
    printf_U(Port, pTarget->lobe_end_ts, NO_PAD);
    printf_U(Port, pTarget->lobe_interval_us, NO_PAD);
    printf_U(Port, pTarget->lobe_angle_deg, NO_PAD | NO_TRAIL);

}

#endif // DECODER_CIS_DEBUG

/**
This function is available even without cis debugging enabled
*/
void print_decoder_cis_debug_data(USART_TypeDef * Port)
{
#ifdef DECODER_CIS_DEBUG

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

#endif // DECODER_CIS_DEBUG
}

