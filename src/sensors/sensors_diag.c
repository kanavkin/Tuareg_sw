#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"


#define USE_SENSORS_DIAG


#ifdef USE_SENSORS_DIAG
VU32 sensors_diag[SNDIAG_COUNT];
#endif // USE_SENSORS_DIAG


/******************************************************************************************************
decoder diag legend
*******************************************************************************************************/

#ifdef USE_SENSORS_DIAG

#define SENSORS_DIAG_LEGEND_LEN 25


const char sensors_diag_legend [SNDIAG_COUNT] [SENSORS_DIAG_LEGEND_LEN] __attribute__((__section__(".rodata"))) = {

"READ_DSENSORS_CALLS",
"ADCIRQ_CALLS",
"ADCIRQ_INJECTEDGR_CALLS",
"DMAIRQ_CALLS",
"DMAIRQ_CH1_CALLS"

};

#endif // USE_SENSORS_DIAG

/******************************************************************************************************
decoder diag logging
*******************************************************************************************************/

void sensors_diag_log_event(sensors_diag_t Event)
{
    #ifdef USE_SENSORS_DIAG

    Assert(Event < SNDIAG_COUNT, 0, 0);

    sensors_diag[Event] += 1;

    #endif // USE_SENSORS_DIAG
}



/******************************************************************************************************
decoder diag printing
*******************************************************************************************************/

exec_result_t print_sensors_diag(USART_TypeDef * Port)
{
    #ifdef USE_SENSORS_DIAG

    U32 cnt;
    exec_result_t result;

    //copy live data to shadow
    result= copy_diag_data(sensors_diag, SNDIAG_COUNT);

    ASSERT_EXEC_OK(result);

    print(Port, "\r\n\r\nSensor Diagnostics: \r\n");

    for(cnt=0; cnt < SNDIAG_COUNT; cnt++)
    {
        //print value and legend text
        printf_U(Port, get_diag_data(cnt), PAD_10);
        print_flash(Port, sensors_diag_legend[cnt]);
        print(Port, "\r\n");
    }

    release_diag_shadow();

    return EXEC_OK;

    #else

    return EXEC_ERROR;

    #endif // USE_SENSORS_DIAG
}



