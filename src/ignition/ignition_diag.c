#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"
#include "ignition_diag.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"

#define USE_IGNITION_DIAG


#ifdef USE_IGNITION_DIAG
VU32 ignition_diag[IGNDIAG_COUNT];
#endif // USE_IGNITION_DIAG


/******************************************************************************************************
ignition diag legend
*******************************************************************************************************/

#ifdef USE_IGNITION_DIAG

#define IGNITION_DIAG_LEGEND_LEN 30


const char ignition_diag_legend [IGNDIAG_COUNT] [IGNITION_DIAG_LEGEND_LEN] __attribute__((__section__(".rodata"))) = {

"CRKPOSH_CALLS",
"CRKPOSH_INHIBIT",
"CRKPOSH_CTRLS_INVALID",
"CRKPOSH_IGNPOS",
"CRKPOSH_IGN1SCHED",
"CRKPOSH_IGN2SCHED",
"CRKPOSH_IGN1_UNPOWER",
"CRKPOSH_IGN2_UNPOWER",
"CRKPOSH_IGN1_POWER",
"CRKPOSH_IGN2_POWER",

"UPDIGNCTRL_CALLS",
"UPDIGNCTRL_REVLIM",
"UPDIGNCTRL_DYN",
"UPDIGNCTRL_DYN_FAIL"

};

#endif // USE_IGNITION_DIAG

/******************************************************************************************************
ignition diag logging
*******************************************************************************************************/

void ignition_diag_log_event(ignition_diag_t Event)
{
    #ifdef USE_IGNITION_DIAG

    VitalAssert(Event < IGNDIAG_COUNT, 0, 0);

    ignition_diag[Event] += 1;

    #endif // USE_IGNITION_DIAG
}



/******************************************************************************************************
ignition diag printing
*******************************************************************************************************/

exec_result_t print_ignition_diag(USART_TypeDef * Port)
{
    #ifdef USE_IGNITION_DIAG

    U32 cnt;
    exec_result_t result;

    //copy live data to shadow
    result= copy_diag_data(ignition_diag, IGNDIAG_COUNT);

    ASSERT_EXEC_OK(result);

    print(Port, "\r\n\r\nIgnition Diagnostics: \r\n");

    for(cnt=0; cnt < IGNDIAG_COUNT; cnt++)
    {
        //print value and legend text
        printf_U(Port, get_diag_data(cnt), PAD_10);
        print_flash(Port, ignition_diag_legend[cnt]);
        print(Port, "\r\n");
    }

    release_diag_shadow();

    return EXEC_OK;

    #else

    return EXEC_ERROR;

    #endif // USE_IGNITION_DIAG
}



