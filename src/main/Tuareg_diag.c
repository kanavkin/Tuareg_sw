#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"
#include "Tuareg_diag.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"

#define USE_TUAREG_DIAG


#ifdef USE_TUAREG_DIAG
VU32 tuareg_diag[TDIAG_COUNT];
#endif // USE_TUAREG_DIAG


/******************************************************************************************************
decoder diag legend
*******************************************************************************************************/

#ifdef USE_TUAREG_DIAG

#define TUAREG_DIAG_LEGEND_LEN 20


const char tuareg_diag_legend [TDIAG_COUNT] [TUAREG_DIAG_LEGEND_LEN] __attribute__((__section__(".rodata"))) = {

    "DECODER_IRQ",
    "DECODER_UPDATE",
    "DECODER_TIMEOUT",
    "DECODER_PASSIVE",

    "IGNITION_IRQ",

    "TRIG_IGN_CALLS",
    "TRIG_COIL_DWELL1",
    "TRIG_COIL_DWELL2",
    "TRIG_COIL_IGN1",
    "TRIG_COIL_IGN2",

    "PROCESSDATA_CALLS",

    "IGNITIONCALC_CALLS",

    "UPDATE_RUNMODE_CALLS",
    "HALTSRC_PRESENT",
    "HALTSRC_CLEAR",

    "ENTER_INIT",
    "ENTER_HALT",
    "RUNNING_HALT_TR",
    "STB_HALT_TR",
    "INIT_HALT_TR",
    "ENTER_RUNNING",
    "CRANKING_RUNNING_TR",
    "HALT_RUNNING_TR",
    "ENTER_STB",
    "RUNNING_STB_TR",
    "CRANKING_STB_TR",
    "HALT_STB_TR",
    "ENTER_CRANKING",
    "ENTER_MTEST",
    "INVALID_RUNMODE"

};

#endif // USE_TUAREG_DIAG

/******************************************************************************************************
decoder diag logging
*******************************************************************************************************/

void tuareg_diag_log_event(tuareg_diag_t Event)
{
    #ifdef USE_TUAREG_DIAG

    VitalAssert(Event < TDIAG_COUNT, 0, 0);

    tuareg_diag[Event] += 1;

    #endif // USE_TUAREG_DIAG
}



/******************************************************************************************************
decoder diag printing
*******************************************************************************************************/

exec_result_t print_tuareg_diag(USART_TypeDef * Port)
{
    #ifdef USE_TUAREG_DIAG

    U32 cnt;
    exec_result_t result;

    //copy live data to shadow
    result= copy_diag_data(tuareg_diag, TDIAG_COUNT);

    ASSERT_EXEC_OK(result);

    print(Port, "\r\n\r\nTuareg Diagnostics: \r\n");

    for(cnt=0; cnt < TDIAG_COUNT; cnt++)
    {
        //print value and legend text
        printf_U(Port, get_diag_data(cnt), PAD_10);
        print_flash(Port, tuareg_diag_legend[cnt]);
        print(Port, "\r\n");
    }

    release_diag_shadow();

    return EXEC_OK;

    #else

    return EXEC_ERROR;

    #endif // USE_TUAREG_DIAG
}



