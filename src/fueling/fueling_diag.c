#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"
#include "fueling_diag.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"

#define USE_FUELING_DIAG


#ifdef USE_FUELING_DIAG
VU32 fueling_diag[FDIAG_COUNT];
#endif // USE_FUELING_DIAG


/******************************************************************************************************
fueling diag legend
*******************************************************************************************************/

#ifdef USE_FUELING_DIAG

#define FUELING_DIAG_LEGEND_LEN 30


const char fueling_diag_legend [FDIAG_COUNT] [FUELING_DIAG_LEGEND_LEN] __attribute__((__section__(".rodata"))) = {

"UPD_CTRLS_CALLS",
"UPD_CTRLS_OP_PRECOND_FAIL",
"UPD_CTRLS_CRANKING",
"UPD_CTRLS_RUNNING",
"UPD_CTRLS_SEQ",
"UPD_CTRLS_BATCH",
"UPD_CTRLS_SPD",
"UPD_CTRLS_ALPHAN",
"UPD_CTRLS_VE_INVAL",
"UPD_CTRLS_AFTTGT_INVAL",

"CRKPOSH_CALLS",
"CRKPOSH_VIT_PRECOND_FAIL",
"CRKPOSH_INJBEG_POS",
"CRKPOSH_SEQ_ERROR",
"CRKPOSH_INJBEG1_SEQ",
"CRKPOSH_INJBEG2_SEQ",
"CRKPOSH_INJBEG_BATCH",

"UPD_BASE_FMAS_CRK_DRY",

"UPD_ACCELCOMP_RPMSCALED",
"UPD_ACCELCOMP_ACCEL",
"UPD_ACCELCOMP_DECEL",
"UPD_ACCELCOMP_TAPERED",

"UPD_TGTFMASS_PRECOND_FAIL",
"UPD_TGTFMASS_ASE_ACT",
"UPD_TGTFMASS_WUE_ACT",
"UPD_TGTFMASS_AE_ACT",
"UPD_TGTFMASS_ZERO"

};

#endif // USE_FUELING_DIAG

/******************************************************************************************************
fueling diag logging
*******************************************************************************************************/

void fueling_diag_log_event(fueling_diag_t Event)
{
    #ifdef USE_FUELING_DIAG

    Assert(Event < FDIAG_COUNT, 0, 0);

    fueling_diag[Event] += 1;

    #endif // USE_FUELING_DIAG
}



/******************************************************************************************************
fueling diag printing
*******************************************************************************************************/

exec_result_t print_fueling_diag(USART_TypeDef * Port)
{
    #ifdef USE_FUELING_DIAG

    U32 cnt;
    exec_result_t result;

    //copy live data to shadow
    result= copy_diag_data(fueling_diag, FDIAG_COUNT);

    ASSERT_EXEC_OK(result);

    print(Port, "\r\n\r\nFueling Diagnostics: \r\n");

    for(cnt=0; cnt < FDIAG_COUNT; cnt++)
    {
        //print value and legend text
        printf_U(Port, get_diag_data(cnt), PAD_10);
        print_flash(Port, fueling_diag_legend[cnt]);
        print(Port, "\r\n");
    }

    release_diag_shadow();

    return EXEC_OK;

    #else

    return EXEC_ERROR;

    #endif // USE_FUELING_DIAG
}



