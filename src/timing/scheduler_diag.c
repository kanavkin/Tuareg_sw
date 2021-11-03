#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"


#define USE_SCHEDULER_DIAG


#ifdef USE_SCHEDULER_DIAG
VU32 scheduler_diag[SCHEDIAG_COUNT];
#endif // USE_SCHEDULER_DIAG


/******************************************************************************************************
scheduler diag legend
*******************************************************************************************************/

#ifdef USE_SCHEDULER_DIAG

#define SCHEDULER_DIAG_LEGEND_LEN 25


const char scheduler_diag_legend [SCHEDIAG_COUNT] [SCHEDULER_DIAG_LEGEND_LEN] __attribute__((__section__(".rodata"))) = {

 "ICH1_RESET",
    "ICH2_RESET",
    "FCH1_RESET",
    "FCH2_RESET",

    "SET_ICH1_1INT",
    "SET_ICH2_1INT",
    "SET_FCH1_1INT",
    "SET_FCH2_1INT",

    "SET_ICH1_2INT",
    "SET_ICH2_2INT",
    "SET_FCH1_2INT",
    "SET_FCH2_2INT",

    "ICH1_REALLOC",
    "ICH2_REALLOC",
    "FCH1_REALLOC",
    "FCH2_REALLOC",

    "ICH1_REALLOC_COMPLETED",
    "ICH2_REALLOC_COMPLETED",
    "FCH1_REALLOC_COMPLETED",
    "FCH2_REALLOC_COMPLETED",

    "ICH1_ALLOC_CUR",
    "ICH2_ALLOC_CUR",
    "FCH1_ALLOC_CUR",
    "FCH2_ALLOC_CUR",

    "ICH1_ALLOC_PREL",
    "ICH2_ALLOC_PREL",
    "FCH1_ALLOC_PREL",
    "FCH2_ALLOC_PREL",

    "ICH1_ALLOC_UPD",
    "ICH2_ALLOC_UPD",
    "FCH1_ALLOC_UPD",
    "FCH2_ALLOC_UPD",

    "ICH1_ALLOC",
    "ICH2_ALLOC",
    "FCH1_ALLOC",
    "FCH2_ALLOC",

    "ICH1_TRIG",
    "ICH2_TRIG",
    "FCH1_TRIG",
    "FCH2_TRIG",

    "DELAY_MININT1",
    "DELAY_MININT2",
    "WRAP"

};

#endif // USE_SCHEDULER_DIAG

/******************************************************************************************************
scheduler diag logging
*******************************************************************************************************/

void scheduler_diag_log_event(scheduler_diag_t Event)
{
    #ifdef USE_SCHEDULER_DIAG

    Assert(Event < SCHEDIAG_COUNT, 0, 0);

    scheduler_diag[Event] += 1;

    #endif // USE_SCHEDULER_DIAG
}



/******************************************************************************************************
scheduler diag printing
*******************************************************************************************************/

exec_result_t print_scheduler_diag(USART_TypeDef * Port)
{
    #ifdef USE_SCHEDULER_DIAG

    U32 cnt;
    exec_result_t result;

    //copy live data to shadow
    result= copy_diag_data(scheduler_diag, SCHEDIAG_COUNT);

    ASSERT_EXEC_OK(result);

    print(Port, "\r\n\r\nScheduler Diagnostics: \r\n");

    for(cnt=0; cnt < SCHEDIAG_COUNT; cnt++)
    {
        //print value and legend text
        printf_U(Port, get_diag_data(cnt), PAD_10);
        print_flash(Port, scheduler_diag_legend[cnt]);
        print(Port, "\r\n");
    }

    release_diag_shadow();

    return EXEC_OK;

    #else

    return EXEC_ERROR;

    #endif // USE_SCHEDULER_DIAG
}



