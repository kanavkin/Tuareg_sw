#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"


#define USE_DECODER_DIAG


#ifdef USE_DECODER_DIAG
VU32 decoder_diag[DDIAG_COUNT];
#endif // USE_DECODER_DIAG


/******************************************************************************************************
decoder diag legend
*******************************************************************************************************/

#ifdef USE_DECODER_DIAG

#define DECODER_DIAG_LEGEND_LEN 20


const char decoder_diag_legend [DDIAG_COUNT] [DECODER_DIAG_LEGEND_LEN] __attribute__((__section__(".rodata"))) = {

"CRK_EXTI_EVENTS",

"CRKPOS_INIT",
"CRKPOS_ASYNC",
"CRKPOS_ASYNC_KEY",
"CRKPOS_ASYNC_GAP",
"CRKPOS_SYNC",

"SYNCHK_ASYN_FAIL",
"SYNCHK_ASYN_PASS",
"SYNCHK_SYN_FAIL",
"SYNCHK_SYN_PASS",

"UPDATE_IRQ_CALLS",
"CRK_NOISEF_EVENTS",
"CAM_NOISEF_EVENTS",

"TIM_UPDATE_EVENTS",
"TIMEOUT_EVENTS",

"CAM_EXTI_EVENTS",
"CISHDL_PRECOND_FAIL",

"ENA_CIS",
"LOBE_BEG",
"LOBE_END",
"INVALID_TRIG",

"CISUPD_CALLS",
"CISUPD_PRECOND_FAIL",
"CISUPD_TRIGGERED",
"CISUPD_PHASE_FAIL",
"CISUPD_PHASE_PASS"

};

#endif // USE_DECODER_DIAG

/******************************************************************************************************
decoder diag logging
*******************************************************************************************************/

void decoder_diag_log_event(decoder_diag_t Event)
{
    #ifdef USE_DECODER_DIAG

    Assert(Event < DDIAG_COUNT, 0, 0);

    decoder_diag[Event] += 1;

    #endif // USE_DECODER_DIAG
}



/******************************************************************************************************
decoder diag printing
*******************************************************************************************************/

exec_result_t print_decoder_diag(USART_TypeDef * Port)
{
    #ifdef USE_DECODER_DIAG

    U32 cnt;
    exec_result_t result;

    //copy live data to shadow
    result= copy_diag_data(decoder_diag, DDIAG_COUNT);

    ASSERT_EXEC_OK(result);

    print(Port, "\r\n\r\nDecoder Diagnostics: \r\n");

    for(cnt=0; cnt < DDIAG_COUNT; cnt++)
    {
        //print value and legend text
        printf_U(Port, get_diag_data(cnt), PAD_10);
        print_flash(Port, decoder_diag_legend[cnt]);
        print(Port, "\r\n");
    }

    release_diag_shadow();

    return EXEC_OK;

    #else

    return EXEC_ERROR;

    #endif // USE_DECODER_DIAG
}



